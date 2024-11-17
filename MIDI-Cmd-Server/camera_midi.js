var easymidi = require('easymidi');
const request = require('request');

async function init() {
    var url = "http://10.10.10.112:8080/";
    var input = new easymidi.Input('NewDay-Stream');
    input.on('noteon', async function (msg) {
    // do something with msg
    if (msg.channel == 1) {
        if (msg.velocity > 0) {
            if (msg.note == 1) {
                await sleep(400);
                request(url + '?cmd=m', { json: true }, (err, res, body) => {});
            }
            if (msg.note == 2) {
                await sleep(400);
                request(url + '?cmd=m2', { json: true }, (err, res, body) => {});
            }
            if (msg.note == 4) {
                await sleep(400);
                request(url + '?cmd=m3', { json: true }, (err, res, body) => {});
            }
            if (msg.note == 3) {
                await sleep(400);
                request(url + '?cmd=m4', { json: true }, (err, res, body) => {});
            }
            console.log(msg);
        }
    }
    });

    // CAM POS PLAN
    // Pos 1 = Blake
    // Pos 2 = Singers
    // Pos 3 = Middle
    // Pos 4 = James
};

function sleep(ms) {
    return new Promise((resolve) => {
        setTimeout(resolve, ms);
    });
}

init();
