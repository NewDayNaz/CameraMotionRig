const http = require('http');
const url = require('url');
const fs = require('fs');

const cmd_file = String(process.env.CAMERA_CMD_FILE) || "";

console.log('Starting up...');
console.log('  CAMERA_CMD_FILE=%s', cmd_file);

http
  .createServer(function (req, res) {
    const q = url.parse(req.url, true).query;
    if (typeof q.cmd !== 'undefined') {
      console.log(q.cmd);
      try {
        fs.writeFileSync(cmd_file, q.cmd);
        // file written successfully
      } catch (err) {
        console.error(err);
      }
    }

    res.writeHead(200, { 'Content-Type': 'text/html' });
    res.end('s');
  })
  .listen(8080);
