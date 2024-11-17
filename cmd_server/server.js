const http = require('http');
const url = require('url');
const fs = require('fs');

const cmd_file = String(process.env.CAMERA_CMD_FILE) || "";

let mCommandsEnabled = true; // State to track whether "m" commands are enabled or disabled

console.log('Starting up...');
console.log('  CAMERA_CMD_FILE=%s', cmd_file);

http
  .createServer(function (req, res) {
    const q = url.parse(req.url, true).query;

    if (typeof q.cmd !== 'undefined') {
      console.log(`Received command: ${q.cmd}`);
      
      // Handle enable/disable for commands starting with "m"
      if (q.cmd === 'enable') {
        mCommandsEnabled = true;
        console.log('Commands starting with "m" have been ENABLED');
      } else if (q.cmd === 'disable') {
        mCommandsEnabled = false;
        console.log('Commands starting with "m" have been DISABLED');
      } else if (q.cmd.startsWith('m')) {
        if (mCommandsEnabled) {
          const modifiedCmd = `t${q.cmd.slice(1)}`; // Replace "m" with "t"
          try {
            fs.writeFileSync(cmd_file, modifiedCmd);
            console.log(`Modified command "${modifiedCmd}" written to file.`);
          } catch (err) {
            console.error('Error writing to file:', err);
          }
        } else {
          console.log(`Command "${q.cmd}" ignored because "m" commands are disabled.`);
        }
      } else {
        try {
          fs.writeFileSync(cmd_file, q.cmd);
          console.log(`Command "${q.cmd}" written to file.`);
        } catch (err) {
          console.error('Error writing to file:', err);
        }
      }
    }

    res.writeHead(200, { 'Content-Type': 'text/html' });
    res.end('s');
  })
  .listen(8080);
