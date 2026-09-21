from __future__ import annotations

import csv
import json
import os
import threading
import time
from dataclasses import asdict
from datetime import datetime
from typing import Callable, Optional

os.environ.setdefault("QT_API", "pyside6")

import numpy as np
import cv2
from PySide6.QtCore import Qt, QThread, QTimer, Signal, Slot
from PySide6.QtGui import QCloseEvent, QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from vision_tuner.analyze import TunerReport, format_suggestions
from vision_tuner.capture import CaptureThread, list_ndi_sources, open_source
from vision_tuner.config import TunerSettings, runs_dir
from vision_tuner.controller import Controller, RigStatus
from vision_tuner.ndi_runtime import ensure_ndi_runtime
from vision_tuner.trials import Recorder, Sample, TrialAborted, TrialRunner
from vision_tuner.vision import VisionEngine, VisionMetrics, overlay


pg.setConfigOptions(antialias=False, background="#14161a", foreground="#d8dce3")


class TrialThread(QThread):
    log = Signal(str)
    finished_report = Signal(object)
    failed = Signal(str)

    def __init__(self, fn: Callable[[], TunerReport]):
        super().__init__()
        self._fn = fn

    def run(self) -> None:
        try:
            report = self._fn()
            self.finished_report.emit(report)
        except TrialAborted as e:
            self.failed.emit(str(e))
        except Exception as e:
            self.failed.emit(f"{type(e).__name__}: {e}")


class MainWindow(QMainWindow):
    frame_ready = Signal()
    status_ready = Signal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera Motion Rig — Vision Tuner")
        self.resize(1480, 920)
        self.settings = TunerSettings.load()
        self.ctl = Controller(self.settings.controller_url)
        self.rec = Recorder()
        self.vision = VisionEngine()
        self._lock = threading.Lock()
        self._frame = None
        self._metrics: Optional[VisionMetrics] = None
        self._status = RigStatus()
        self._capture: Optional[CaptureThread] = None
        self._trial: Optional[TrialThread] = None
        self._abort = threading.Event()
        self._status_stop = threading.Event()
        self._t0 = time.perf_counter()
        self._live_n = 1200
        self._t = np.zeros(self._live_n)
        self._flow = np.zeros(self._live_n)
        self._jit = np.zeros(self._live_n)
        self._pan = np.zeros(self._live_n)
        self._tilt = np.zeros(self._live_n)
        self._zoom = np.zeros(self._live_n)
        self._cmd = np.zeros(self._live_n)
        self._i = 0
        self._filled = 0
        self._last_preview = 0.0
        self._plot_tick = 0
        self._report: Optional[TunerReport] = None

        self._build()
        self.frame_ready.connect(self._show_frame)
        self.status_ready.connect(self._show_status)

        self._ui_timer = QTimer(self)
        self._ui_timer.setInterval(120)
        self._ui_timer.timeout.connect(self._tick_plots)
        self._ui_timer.start()

        self._status_thread = threading.Thread(target=self._status_loop, daemon=True, name="status")
        self._status_thread.start()
        self._log("Ready. Connect the controller, pick NDI (or Demo), then run a trial.")
        self._log("Automations are muted while a trial runs. STOP is always available.")
        self._log(ensure_ndi_runtime().summary())

    def _build(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        top = QHBoxLayout()
        self.url = QLineEdit(self.settings.controller_url)
        self.url.setPlaceholderText("http://10.0.0.12")
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.connect_controller)
        self.source_kind = QComboBox()
        self.source_kind.addItems(["ndi", "webcam", "demo"])
        self.source_kind.setCurrentText(self.settings.video_source)
        self.ndi_combo = QComboBox()
        self.ndi_combo.setMinimumWidth(260)
        self.ndi_combo.setEditable(True)
        if self.settings.ndi_source:
            self.ndi_combo.addItem(self.settings.ndi_source)
        self.btn_ndi = QPushButton("Find NDI")
        self.btn_ndi.clicked.connect(self.refresh_ndi)
        self.btn_video = QPushButton("Start video")
        self.btn_video.clicked.connect(self.toggle_video)
        self.lbl_link = QLabel("controller: —")
        self.lbl_video = QLabel("video: —")
        for w in (
            QLabel("Controller"),
            self.url,
            self.btn_connect,
            QLabel("Video"),
            self.source_kind,
            self.ndi_combo,
            self.btn_ndi,
            self.btn_video,
            self.lbl_link,
            self.lbl_video,
        ):
            top.addWidget(w)
        top.addStretch(1)
        layout.addLayout(top)

        self.lbl_hud = QLabel("PAN —   TILT —   ZOOM —")
        self.lbl_hud.setStyleSheet("font-family: Consolas, monospace; font-size: 13px; padding: 4px;")
        layout.addWidget(self.lbl_hud)

        split = QSplitter(Qt.Orientation.Horizontal)
        layout.addWidget(split, 1)

        left = QWidget()
        left_l = QVBoxLayout(left)
        self.preview = QLabel("No video")
        self.preview.setMinimumSize(640, 360)
        self.preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview.setStyleSheet("background: #0b0c0e; color: #888;")
        left_l.addWidget(self.preview, 1)
        split.addWidget(left)

        right = QWidget()
        right_l = QVBoxLayout(right)
        right_l.addWidget(self._controls())
        self.tabs = QTabWidget()
        self.tabs.addTab(self._live_plots(), "Live motion")
        self.tabs.addTab(self._sweep_plots(), "Speed sweep")
        self.tabs.addTab(self._range_plots(), "Range / GOTO")
        self.suggest = QPlainTextEdit()
        self.suggest.setReadOnly(True)
        self.suggest.setPlaceholderText("Trial suggestions land here.")
        self.tabs.addTab(self.suggest, "Suggestions")
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.tabs.addTab(self.log_view, "Log")
        right_l.addWidget(self.tabs, 1)
        split.addWidget(right)
        split.setSizes([720, 760])

    def _controls(self) -> QWidget:
        box = QGroupBox("Trials and jog")
        grid = QGridLayout(box)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setStyleSheet("background: #8b1e1e; color: white; font-weight: 700; padding: 8px;")
        self.btn_stop.clicked.connect(self.emergency_stop)
        self.btn_home = QPushButton("HOME")
        self.btn_home.clicked.connect(lambda: self._start_trial("HOME", self._job_home))
        self.btn_still = QPushButton("Stillness")
        self.btn_still.clicked.connect(lambda: self._start_trial("stillness", self._job_still))
        self.btn_sweep_pan = QPushButton("Sweep pan")
        self.btn_sweep_tilt = QPushButton("Sweep tilt")
        self.btn_sweep_zoom = QPushButton("Sweep zoom")
        self.btn_sweep_pan.clicked.connect(lambda: self._start_trial("sweep pan", lambda: self._job_sweep("pan")))
        self.btn_sweep_tilt.clicked.connect(lambda: self._start_trial("sweep tilt", lambda: self._job_sweep("tilt")))
        self.btn_sweep_zoom.clicked.connect(lambda: self._start_trial("sweep zoom", lambda: self._job_sweep("zoom")))
        self.btn_range_pan = QPushButton("Range pan")
        self.btn_range_tilt = QPushButton("Range tilt")
        self.btn_range_zoom = QPushButton("Range zoom")
        self.btn_range_pan.clicked.connect(lambda: self._start_trial("range pan", lambda: self._job_range("pan")))
        self.btn_range_tilt.clicked.connect(lambda: self._start_trial("range tilt", lambda: self._job_range("tilt")))
        self.btn_range_zoom.clicked.connect(lambda: self._start_trial("range zoom", lambda: self._job_range("zoom")))
        self.btn_goto = QPushButton("GOTO duration")
        self.btn_goto.clicked.connect(lambda: self._start_trial("GOTO", self._job_goto))
        self.btn_irun = QPushButton("IRUN search (pan)")
        self.btn_irun.clicked.connect(lambda: self._start_trial("IRUN", self._job_irun))
        self.btn_pt_scale = QPushButton("Tele pan scale")
        self.btn_pt_scale.clicked.connect(lambda: self._start_trial("tele pan scale", self._job_pt_scale))
        self.btn_repeat = QPushButton("Preset repeat")
        self.btn_repeat.clicked.connect(self._run_repeat)
        self.btn_full = QPushButton("Full auto suite")
        self.btn_full.setStyleSheet("font-weight: 700; padding: 8px;")
        self.btn_full.clicked.connect(self._run_full)
        self.btn_export = QPushButton("Export run")
        self.btn_export.clicked.connect(self.export_run)
        self.btn_abort = QPushButton("Abort trial")
        self.btn_abort.clicked.connect(self.abort_trial)

        btns = [
            self.btn_stop, self.btn_home, self.btn_abort,
            self.btn_still, self.btn_sweep_pan, self.btn_sweep_tilt, self.btn_sweep_zoom,
            self.btn_range_pan, self.btn_range_tilt, self.btn_range_zoom,
            self.btn_goto, self.btn_irun, self.btn_pt_scale, self.btn_repeat, self.btn_full, self.btn_export,
        ]
        for i, b in enumerate(btns):
            grid.addWidget(b, i // 4, i % 4)

        jog = QHBoxLayout()
        jog.addWidget(QLabel("Hold to jog (travel-safe):"))
        for axis, label, vel in (
            ("pan", "Pan −", -120),
            ("pan", "Pan +", 120),
            ("tilt", "Tilt −", -120),
            ("tilt", "Tilt +", 120),
            ("zoom", "Wide", -80),
            ("zoom", "Tele", 80),
        ):
            b = QPushButton(label)
            b.pressed.connect(lambda a=axis, v=vel: self._jog(a, v))
            b.released.connect(self.emergency_stop)
            jog.addWidget(b)
        grid.addLayout(jog, 4, 0, 1, 4)
        return box

    def _live_plots(self) -> QWidget:
        w = pg.GraphicsLayoutWidget()
        self.p_flow = w.addPlot(row=0, col=0, title="Image speed (slide or zoom-scale px/s)")
        self.p_jit = w.addPlot(row=1, col=0, title="Flow jitter (px)")
        self.p_pos = w.addPlot(row=2, col=0, title="Firmware position (steps)")
        self.p_cmd = w.addPlot(row=3, col=0, title="|Commanded velocity| (step/s)")
        for p in (self.p_flow, self.p_jit, self.p_pos, self.p_cmd):
            p.showGrid(x=True, y=True, alpha=0.2)
            p.setLabel("bottom", "t (s)")
            try:
                p.setDownsampling(mode="peak", auto=True)
                p.setClipToView(True)
            except Exception:
                pass
            p.enableAutoRange(x=False)
        live_pen = lambda c: pg.mkPen(c, width=1)
        self.c_flow = self._live_curve(self.p_flow, live_pen("#6ee7b7"))
        self.c_jit = self._live_curve(self.p_jit, live_pen("#fbbf24"))
        self.c_pan = self._live_curve(self.p_pos, live_pen("#93c5fd"), name="pan")
        self.c_tilt = self._live_curve(self.p_pos, live_pen("#f9a8d4"), name="tilt")
        self.c_zoom = self._live_curve(self.p_pos, live_pen("#fdba74"), name="zoom")
        self.p_pos.addLegend(offset=(8, 8))
        self.c_cmd = self._live_curve(self.p_cmd, live_pen("#c4b5fd"))
        return w

    def _live_curve(self, plot, pen, name=None):
        kw = {"pen": pen}
        if name:
            kw["name"] = name
        c = plot.plot(**kw)
        try:
            c.setClipToView(True)
            c.setDownsampling(auto=True, method="peak")
        except Exception:
            pass
        return c

    def _sweep_plots(self) -> QWidget:
        w = pg.GraphicsLayoutWidget()
        self.p_pxs = w.addPlot(row=0, col=0, title="Observed px/s vs commanded step/s")
        self.p_jr = w.addPlot(row=0, col=1, title="Jitter ratio vs speed (lower is smoother)")
        self.p_pps = w.addPlot(row=1, col=0, colspan=2, title="Pixels per step vs speed (drop = lost steps)")
        for p in (self.p_pxs, self.p_jr, self.p_pps):
            p.showGrid(x=True, y=True, alpha=0.2)
            p.setLabel("bottom", "commanded step/s")
            p.addLegend()
        self.sweep_curves = {}
        colors = {"pan": "#93c5fd", "tilt": "#f9a8d4", "zoom": "#fdba74"}
        for axis, col in colors.items():
            self.sweep_curves[axis] = (
                self.p_pxs.plot(pen=pg.mkPen(col, width=2), symbol="o", name=axis),
                self.p_jr.plot(pen=pg.mkPen(col, width=2), symbol="o", name=axis),
                self.p_pps.plot(pen=pg.mkPen(col, width=2), symbol="o", name=axis),
            )
        return w

    def _range_plots(self) -> QWidget:
        w = pg.GraphicsLayoutWidget()
        self.p_range = w.addPlot(row=0, col=0, title="Image speed vs firmware position (range crawl)")
        self.p_goto = w.addPlot(row=1, col=0, title="GOTO: image px/s vs time")
        self.p_range.showGrid(x=True, y=True, alpha=0.2)
        self.p_goto.showGrid(x=True, y=True, alpha=0.2)
        self.p_range.setLabel("bottom", "position (steps)")
        self.p_range.setLabel("left", "px/s")
        self.p_goto.setLabel("bottom", "t (s)")
        self.p_goto.addLegend()
        self.c_goto = self.p_goto.plot(pen=pg.mkPen("#6ee7b7", width=2), name="px/s")
        return w

    def _runner(self) -> TrialRunner:
        return TrialRunner(
            self.ctl,
            self.rec,
            lambda: self._metrics,
            self._abort.is_set,
            lambda msg: self._trial.log.emit(msg) if self._trial else self._log(msg),
            latest_gray=self.vision.snapshot_gray,
        )

    def _job_home(self) -> TunerReport:
        r = self._runner()
        r.wait_home()
        return r.report

    def _job_still(self) -> TunerReport:
        r = self._runner()
        r.stillness_baseline()
        return r.report

    def _job_sweep(self, axis: str) -> TunerReport:
        r = self._runner()
        self.ctl.mute_automations()
        try:
            r.stillness_baseline()
            r.velocity_sweep(axis)
        finally:
            self.ctl.stop()
        return r.report

    def _job_range(self, axis: str) -> TunerReport:
        r = self._runner()
        self.ctl.mute_automations()
        try:
            r.stillness_baseline()
            r.range_crawl(axis)
        finally:
            self.ctl.stop()
        return r.report

    def _job_goto(self) -> TunerReport:
        r = self._runner()
        r.stillness_baseline()
        r.goto_duration()
        return r.report

    def _job_irun(self) -> TunerReport:
        r = self._runner()
        self.ctl.mute_automations()
        try:
            r.stillness_baseline()
            r.irun_search("pan", 450)
        finally:
            self.ctl.stop()
        return r.report

    def _job_pt_scale(self) -> TunerReport:
        r = self._runner()
        self.ctl.mute_automations()
        try:
            r.stillness_baseline()
            r.zoom_pt_scale_check(450)
        finally:
            self.ctl.stop()
        return r.report

    def _job_repeat(self) -> TunerReport:
        r = self._runner()
        r.preset_repeatability()
        return r.report

    def _job_full(self) -> TunerReport:
        r = self._runner()
        return r.full_suite()

    def _start_trial(self, name: str, fn: Callable[[], TunerReport]) -> None:
        if self._trial and self._trial.isRunning():
            QMessageBox.information(self, "Busy", "A trial is already running. Abort it first.")
            return
        self._abort.clear()
        self._log(f"--- {name} ---")
        self._trial = TrialThread(fn)
        self._trial.log.connect(self._log)
        self._trial.finished_report.connect(self._trial_done)
        self._trial.failed.connect(self._trial_fail)
        self._trial.start()

    def _run_full(self) -> None:
        if QMessageBox.question(
            self,
            "Full suite",
            "This HOMEs if needed, mutes automations, sweeps pan/tilt/zoom, "
            "crawls zoom only, GOTO a preset, then pan IRUN and tele-vs-wide pan scale.\n\n"
            "Stay near the rig. Abort or STOP if anything binds.",
        ) != QMessageBox.StandardButton.Yes:
            return
        self._start_trial("full suite", self._job_full)

    def _run_repeat(self) -> None:
        if QMessageBox.question(
            self,
            "Preset repeatability",
            "GOTO the first saved preset many times: pan, then tilt, then zoom, "
            "then pan+tilt, then all three.\n\n"
            "Each stage leaves the shot (~260 pan/tilt or ~140 zoom steps) from below "
            "and from above (backlash take-up), then recalls.\n\n"
            "Does not overwrite presets. Stay near the rig. Video must be running.",
        ) != QMessageBox.StandardButton.Yes:
            return
        self._start_trial("preset repeat", self._job_repeat)

    @Slot(object)
    def _trial_done(self, report: TunerReport) -> None:
        self._finish_trial()
        self._apply_report(report)
        self._log("Trial finished.")

    @Slot(str)
    def _trial_fail(self, msg: str) -> None:
        self._finish_trial()
        self._log(f"Trial ended: {msg}")
        if self._report:
            self._apply_report(self._report)

    def _finish_trial(self) -> None:
        try:
            self.ctl.stop()
        except Exception:
            pass
        try:
            self.ctl.restore_automations()
        except Exception as e:
            self._log(f"Could not restore automations: {e}")

    def _apply_report(self, report: TunerReport) -> None:
        if self._report and report is not self._report:
            stillness_only = (
                not report.sweep and not report.ranges
                and report.goto is None and not report.irun
                and report.zoom_pt_scale is None
                and report.repeatability is None
            )
            if not stillness_only:
                if report.sweep:
                    report.sweep = list(self._report.sweep) + list(report.sweep)
                    report.sweep = _unique_sweep(report.sweep)
                else:
                    report.sweep = list(self._report.sweep)
                if report.ranges:
                    latest = {}
                    for r in list(self._report.ranges) + list(report.ranges):
                        latest[r.axis] = r
                    report.ranges = [latest[a] for a in ("pan", "tilt", "zoom") if a in latest]
                if report.goto is None:
                    report.goto = self._report.goto
                if not report.irun:
                    report.irun = list(self._report.irun)
                if report.zoom_pt_scale is None:
                    report.zoom_pt_scale = self._report.zoom_pt_scale
                if report.repeatability is None:
                    report.repeatability = self._report.repeatability
                if report.latency_s <= 0:
                    report.latency_s = self._report.latency_s
                report.notes = list(dict.fromkeys(self._report.notes + report.notes))
            if report.stillness_floor_px_s <= 0 and self._report.stillness_floor_px_s > 0:
                report.stillness_floor_px_s = self._report.stillness_floor_px_s
            if report.stillness_floor <= 0 and self._report.stillness_floor > 0:
                report.stillness_floor = self._report.stillness_floor
            if report.stillness_zoom_px_s <= 0 and self._report.stillness_zoom_px_s > 0:
                report.stillness_zoom_px_s = self._report.stillness_zoom_px_s
        self._report = report
        self.suggest.setPlainText(format_suggestions(report))
        self.tabs.setCurrentWidget(self.suggest)
        self._draw_sweep(report)
        self._draw_range_goto(report)
        self.tabs.setCurrentIndex(3)

    def _draw_sweep(self, report: TunerReport) -> None:
        for axis, curves in self.sweep_curves.items():
            pts = sorted((p for p in report.sweep if p.axis == axis), key=lambda p: p.speed)
            if not pts:
                continue
            sp = [p.speed for p in pts]
            curves[0].setData(sp, [p.mean_px_s for p in pts])
            curves[1].setData(sp, [p.jitter_ratio for p in pts])
            curves[2].setData(sp, [p.px_per_step for p in pts])

    def _draw_range_goto(self, report: TunerReport) -> None:
        self.p_range.clear()
        self.p_range.addLegend()
        colors = {"pan": "#93c5fd", "tilt": "#f9a8d4", "zoom": "#fdba74"}
        for r in report.ranges:
            xs, ys = [], []
            for s in self.rec.samples:
                if s.trial.startswith(f"range-{r.axis}"):
                    xs.append(getattr(s, r.axis))
                    ys.append(s.vis_px_s(r.axis))
            if xs:
                if len(xs) > 600:
                    k = max(1, len(xs) // 600)
                    xs, ys = xs[::k], ys[::k]
                self.p_range.plot(xs, ys, pen=pg.mkPen(colors[r.axis], width=2), name=f"{r.axis} {r.reason}")
        self.p_goto.clear()
        self.p_goto.addLegend()
        self.c_goto = self.p_goto.plot(pen=pg.mkPen("#6ee7b7", width=2), name="px/s")
        if report.goto:
            t0 = None
            ts, vs = [], []
            for s in self.rec.samples:
                if s.trial.startswith("goto-"):
                    if t0 is None:
                        t0 = s.t
                    ts.append(s.t - t0)
                    vs.append(s.vis_px_s())
            if ts:
                if len(ts) > 600:
                    k = max(1, len(ts) // 600)
                    ts, vs = ts[::k], vs[::k]
                self.c_goto.setData(ts, vs)
                g = report.goto
                dash = pg.mkPen("#93c5fd", style=Qt.PenStyle.DashLine)
                pink = pg.mkPen("#f9a8d4", style=Qt.PenStyle.DashLine)
                orange = pg.mkPen("#fdba74", style=Qt.PenStyle.DotLine)
                self.p_goto.addItem(pg.InfiniteLine(pos=g.visual_start_s, angle=90, pen=dash))
                self.p_goto.addItem(pg.InfiniteLine(pos=g.visual_end_s, angle=90, pen=pink))
                if g.duration_s:
                    self.p_goto.addItem(pg.InfiniteLine(pos=g.duration_s, angle=90, pen=orange))

    def _jog(self, axis: str, vel: float) -> None:
        v = {"pan": 0.0, "tilt": 0.0, "zoom": 0.0}
        v[axis] = vel
        try:
            self.ctl.set_velocity(**v)
        except Exception as e:
            self._log(f"jog failed: {e}")

    def emergency_stop(self) -> None:
        self._abort.set()
        try:
            self.ctl.stop()
        except Exception as e:
            self._log(f"STOP failed: {e}")

    def abort_trial(self) -> None:
        self._abort.set()
        self.emergency_stop()
        self._log("Abort requested")

    def connect_controller(self) -> None:
        self.ctl.set_url(self.url.text().strip())
        self.settings.controller_url = self.ctl.base_url
        try:
            self.settings.save()
        except OSError as e:
            self._log(f"Could not save settings: {e}")
        try:
            st = self.ctl.ping()
        except Exception as e:
            self.lbl_link.setText("controller: error")
            self._log(f"Connect failed: {e}")
            return
        if not st.ok:
            self.lbl_link.setText("controller: error")
            self._log(f"Connect failed: {st.message}")
            return
        self.lbl_link.setText("controller: ok")
        self._show_status(st)
        self._log(f"Connected {self.ctl.base_url}  homed={st.homed} auto={st.preset_recall}")

    def refresh_ndi(self) -> None:
        rt = ensure_ndi_runtime()
        names, err = list_ndi_sources()
        self.ndi_combo.clear()
        if names:
            self.ndi_combo.addItems(names)
            self._log(f"{rt.summary()}")
            self._log(f"NDI sources: {', '.join(names)}")
        else:
            self._log(err or "No NDI sources")
            QMessageBox.information(self, "NDI", err or "No NDI sources found.")

    def toggle_video(self) -> None:
        if self._capture:
            self._capture.stop()
            self._capture.join(timeout=1.5)
            self._capture = None
            self.btn_video.setText("Start video")
            self.lbl_video.setText("video: stopped")
            return
        kind = self.source_kind.currentText()
        name = self.ndi_combo.currentText().strip()
        self.settings.video_source = kind
        self.settings.ndi_source = name
        try:
            self.settings.save()
        except OSError as e:
            self._log(f"Could not save settings: {e}")
        try:
            src = open_source(kind, name, self.settings.webcam_index, lambda: self.ctl.last_cmd)
        except Exception as e:
            QMessageBox.warning(self, "Video", str(e))
            self._log(f"Video open failed: {e}")
            return
        self.vision.reset()
        self._capture = CaptureThread(src, self._on_frame)
        self._capture.start()
        self.btn_video.setText("Stop video")
        self.lbl_video.setText(f"video: {kind}")
        self._log(f"Video started ({kind} {name})")

    def _on_frame(self, frame, t: float) -> None:
        m = self.vision.process(frame, t)
        st = self.ctl.last_status
        cmd = self.ctl.last_cmd
        sample = Sample(
            t=t,
            pan=st.pan,
            tilt=st.tilt,
            zoom=st.zoom,
            cmd_pan=cmd["pan"],
            cmd_tilt=cmd["tilt"],
            cmd_zoom=cmd["zoom"],
            flow_dx=m.flow_dx,
            flow_dy=m.flow_dy,
            flow_mag=m.flow_mag,
            jitter=m.jitter,
            radial=m.radial,
            px_per_s=m.px_per_s,
            zoom_px_s=m.zoom_px_s,
            sharpness=m.sharpness,
            texture=m.texture,
            low_texture=m.low_texture,
            still=m.still,
            moving=st.moving,
            homing=st.homing,
            fault=st.any_fault,
            pwm_hit=st.pwm_hit,
        )
        self.rec.add(sample)
        i = self._i % self._live_n
        self._t[i] = t - self._t0
        self._flow[i] = max(m.px_per_s, m.zoom_px_s)
        self._jit[i] = m.jitter
        self._pan[i] = st.pan
        self._tilt[i] = st.tilt
        self._zoom[i] = st.zoom
        self._cmd[i] = abs(cmd["pan"]) + abs(cmd["tilt"]) + abs(cmd["zoom"])
        self._i += 1
        self._filled = min(self._filled + 1, self._live_n)
        hud = {
            "P": f"{st.pan:.0f}",
            "T": f"{st.tilt:.0f}",
            "Z": f"{st.zoom:.0f}",
            "cmd": f"{cmd['pan']:.0f},{cmd['tilt']:.0f},{cmd['zoom']:.0f}",
        }
        now = time.perf_counter()
        paint = now - self._last_preview >= 0.09
        vis = None
        if paint:
            pv = frame
            if pv.shape[1] > 960:
                s = 960.0 / float(pv.shape[1])
                pv = cv2.resize(
                    pv,
                    (960, max(1, int(pv.shape[0] * s))),
                    interpolation=cv2.INTER_AREA,
                )
            vis = overlay(pv, m, hud)
            self._last_preview = now
        with self._lock:
            self._metrics = m
            if vis is not None:
                self._frame = vis
        if paint:
            self.frame_ready.emit()

    @Slot()
    def _show_frame(self) -> None:
        with self._lock:
            frame = self._frame
        if frame is None:
            return
        h, w = frame.shape[:2]
        img = QImage(frame.data, w, h, frame.strides[0], QImage.Format.Format_BGR888).copy()
        pix = QPixmap.fromImage(img).scaled(
            self.preview.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation,
        )
        self.preview.setPixmap(pix)

    def _status_loop(self) -> None:
        while not self._status_stop.is_set():
            try:
                st = self.ctl.positions()
                if st.ok:
                    self.status_ready.emit(st)
            except Exception:
                pass
            time.sleep(0.12)

    @Slot(object)
    def _show_status(self, st: RigStatus) -> None:
        self._status = st
        fault = " FAULT" if st.any_fault else ""
        home = "HOMING" if st.homing else ("HOMED" if st.homed else "NOT HOMED")
        auto = "AUTO ON" if st.preset_recall else "AUTO OFF"
        pwm = " PWM" if st.pwm_hit else ""
        err = f"  {st.error}" if st.error else ""
        zp = f"{st.zoom_pct:.0f}%" if st.zoom_pct is not None else "—"
        self.lbl_hud.setText(
            f"PAN {st.pan:7.0f}   TILT {st.tilt:7.0f}   ZOOM {st.zoom:7.0f} ({zp})   "
            f"{home}  {auto}{fault}{pwm}{err}"
        )
        if st.ok:
            self.lbl_link.setText("controller: ok")

    def _tick_plots(self) -> None:
        n = self._filled
        if n < 4:
            return
        if self.tabs.currentIndex() != 0:
            return
        i = self._i
        step = max(1, n // 280)
        sl = np.arange(i - n, i, step)
        idx = sl % self._live_n
        t = self._t[idx]
        self.c_flow.setData(t, self._flow[idx])
        self.c_jit.setData(t, self._jit[idx])
        self.c_pan.setData(t, self._pan[idx])
        self.c_tilt.setData(t, self._tilt[idx])
        self.c_zoom.setData(t, self._zoom[idx])
        self.c_cmd.setData(t, self._cmd[idx])
        self._plot_tick += 1
        if self._plot_tick % 8 == 1:
            for p in (self.p_flow, self.p_jit, self.p_pos, self.p_cmd):
                p.enableAutoRange(axis="y", enable=True)
                p.enableAutoRange(axis="y", enable=False)

    def export_run(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        folder = runs_dir() / stamp
        folder.mkdir(parents=True, exist_ok=True)
        samples = list(self.rec.samples)
        csv_path = folder / "samples.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            fields = list(Sample.__dataclass_fields__.keys())
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for s in samples:
                w.writerow({k: getattr(s, k) for k in fields})
        if self._report:
            (folder / "report.json").write_text(json.dumps(asdict(self._report), indent=2), encoding="utf-8")
            (folder / "suggestions.txt").write_text(format_suggestions(self._report), encoding="utf-8")
        self._log(f"Exported {folder}")
        QMessageBox.information(self, "Export", f"Wrote {folder}")

    @Slot(str)
    def _log(self, msg: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_view.appendPlainText(f"{ts}  {msg}")

    def closeEvent(self, event: QCloseEvent) -> None:
        self._abort.set()
        self._status_stop.set()
        try:
            self.ctl.stop()
        except Exception:
            pass
        if self._capture:
            self._capture.stop()
        event.accept()


def _unique_sweep(points):
    seen = {}
    for p in points:
        seen[(p.axis, p.speed)] = p
    return list(seen.values())


def main() -> int:
    app = QApplication([])
    app.setApplicationName("Vision Tuner")
    win = MainWindow()
    win.show()
    return app.exec()
