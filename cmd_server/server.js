const http = require('http');
const url = require('url');
const fs = require('fs');

http
  .createServer(function (req, res) {
    const q = url.parse(req.url, true).query;
    if (typeof q.cmd !== 'undefined') {
      console.log(q.cmd);
      try {
        fs.writeFileSync('/home/pi/camera_async/send_cmd.txt', q.cmd);
        // file written successfully
      } catch (err) {
        console.error(err);
      }
    }

    res.writeHead(200, { 'Content-Type': 'text/html' });
    res.end('s');
  })
  .listen(8080);
