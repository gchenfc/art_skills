// const HOST = '192.168.0.15'
// const HOST = '143.215.91.93'
// const HOST = '172.20.10.2'
const HOST = '143.215.91.135'
const websocket = new WebSocket("ws://"+HOST+":5900/");

const [W, H] = [2.9464, 2.26];

// Reference source: https://github.com/shuding/apple-pencil-safari-api-test
const $force = document.querySelectorAll('#force')[0]
const $touches = document.querySelectorAll('#touches')[0]
const canvas = document.querySelectorAll('canvas')[0]
const context = canvas.getContext('2d')
const canvas_fit = document.querySelectorAll('canvas')[1]
const context_fit = canvas_fit.getContext('2d')
const canvas_touch = document.querySelectorAll('canvas')[1]
let lineWidth = 0
let isMousedown = false
let points = []
let startTime = window.performance.now()
function t() { return (window.performance.now() - startTime) / 1000; }

canvas.width = window.innerWidth * 2
canvas.height = window.innerHeight * 2
canvas_fit.width = window.innerWidth * 2
canvas_fit.height = window.innerHeight * 2
let scale = 1;

// robot.send('xLl0.9')
// robot.send('xLd0.63')
// robot.send('xLr2.5964')
// robot.send('xLu1.91')
const [xmin, xmax] = [0.9, 2.5964];
const [ymin, ymax] = [0.63, 1.91];

const strokeHistory = []
const colorHistory = []

const requestIdleCallback = window.requestIdleCallback || function (fn) { setTimeout(fn, 1) };

function clamp(xy, xbnds, ybnds) {
  return [
    Math.min(Math.max(xy[0], xbnds[0]), xbnds[1]),
    Math.min(Math.max(xy[1], ybnds[0]), ybnds[1]),
  ];
}
function convert_xy_to_normalized(x, y) {
  // return [x / canvas.width, y / canvas.width];

  y = canvas.height - y;
  return clamp(
    [
      (x - canvas.width / 2) / scale + (xmin + xmax) / 2,
      (y - canvas.height / 2) / scale + (ymin + ymax) / 2,
    ],
    [xmin, xmax],
    [ymin, ymax]
  );
}
function convert_xy_from_normalized(x, y) {
  const ret = [
    (x - (xmin + xmax) / 2) * scale + canvas.width / 2,
    (y - (ymin + ymax) / 2) * scale + canvas.height / 2,
  ];
  ret[1] = canvas.height - ret[1];
  return ret;
}
// function convert_raw_to_xy(x_raw, y_raw) {
//   return []
// }

websocket.onmessage = function (event) {
  console.log(event.data);
  let command = event.data[0];
  xy = event.data.slice(1).split(',')
  x_raw = parseFloat(xy[0])
  y_raw = parseFloat(xy[1])
  x = x_raw * canvas.width
  y = y_raw * canvas.width
  console.log(command, x, y)
  context_fit.strokeStyle = 'green'
  context_fit.fillStyle = 'green'
  context_fit.lineCap = 'round'
  context_fit.lineJoin = 'round'
  context_fit.lineWidth = 10
  if (command == 'M') {
    // context_fit.beginPath();
    // context_fit.ellipse(x, y, 10, 10, 0, 0, 2 * Math.PI);
    // context_fit.fill();
    // context_fit.stroke();
    context_fit.moveTo(x, y);
  } else if (command == 'L') {
    context_fit.lineTo(x, y);
    context_fit.stroke();
  } else if (command == 'U') {
  } else if (command == 'F') {
    // Draw CDPR bounds
    context_fit.beginPath();
    context_fit.strokeStyle = 'black'
    context_fit.fillStyle = 'black'
    // aspect_ratio = x_raw / y_raw;
    // console.log(aspect_ratio, canvas.width, canvas.height);
    // if (canvas.width < canvas.height * aspect_ratio) {
    //   // context_fit.fillRect(0, canvas.width / aspect_ratio, canvas.width, canvas.height);
    // } else {
    //   // context_fit.fillRect(canvas.height * aspect_ratio, 0, canvas.width, canvas.height);
    // }
    // context_fit.stroke();

    const aspect_ratio = (xmax - xmin) / (ymax - ymin);
    const canvas_aspect_ratio = canvas.width / canvas.height;
    if (canvas_aspect_ratio < aspect_ratio) {  // ipad is too narrow
      scale = canvas.width / (xmax - xmin);
    } else {
      scale = canvas.height / (ymax - ymin);
    }

    context_fit.beginPath();
    context_fit.strokeStyle = 'black';
    context_fit.fillStyle = 'black';
    context_fit.fillRect(0, 0, canvas.width, canvas.height);
    context_fit.stroke();
    context_fit.beginPath();
    context_fit.strokeStyle = 'green';
    lr = convert_xy_from_normalized(xmin, ymin);
    tr = convert_xy_from_normalized(xmax, ymax);
    console.log(lr, tr);
    context_fit.clearRect(lr[0], lr[1], (tr[0] - lr[0]), (tr[1] - lr[1]));
    context_fit.rect(lr[0], lr[1], (tr[0] - lr[0]), (tr[1] - lr[1]));
    context_fit.stroke();
  }
};

/**
 * This function takes in an array of points and draws them onto the canvas.
 * @param {array} stroke array of points to draw on the canvas
 * @param {String} color string which will only temporarily set the color for a stroke
 * @return {void}
 */
function drawOnCanvas(stroke, color='') {
  if (color.length > 0) {
    var previousColor = context.strokeStyle; 
    context.strokeStyle = color;
  }
  context.lineCap = 'round'
  context.lineJoin = 'round'

  const l = stroke.length - 1
  if (stroke.length >= 3) {
    // const xc = (stroke[l].x + stroke[l - 1].x) / 2
    // const yc = (stroke[l].y + stroke[l - 1].y) / 2
    context.lineWidth = stroke[l - 1].lineWidth * 10
    // context.lineWidth = 150;
    // context.quadraticCurveTo(stroke[l - 1].x, stroke[l - 1].y, xc, yc)
    context.lineTo(stroke[l].x, stroke[l].y)
    context.stroke()
    context.beginPath()
    context.moveTo(stroke[l].x, stroke[l].y)
    // context.moveTo(xc, yc)
  } else {
    const point = stroke[l];
    context.lineWidth = point.lineWidth * 10
    // context.lineWidth = 150;
    context.strokeStyle = point.color
    context.beginPath()
    context.moveTo(point.x, point.y)
    context.stroke()
  }

  if (color.length > 0) {
    context.strokeStyle = previousColor;
  }
}

/**
 * Remove the previous stroke from history and repaint the entire canvas based on history
 * @return {void}
 */
function undoDraw() {
  strokeHistory.pop()
  colorHistory.pop()
  context.clearRect(0, 0, canvas.width, canvas.height)

  strokeHistory.map(function (stroke, ind) {
    if (strokeHistory.length === 0) return

    context.beginPath()

    let strokePath = [];
    stroke.map(function (point) {
      strokePath.push(point)
      drawOnCanvas(strokePath, colorHistory[ind])
    })
  })
}

function changeBlack() {
  if (context.strokeStyle != 'black') {
    context.strokeStyle = 'black';
  }
  sendColorChange();
}
function changeRed() {
  if (context.strokeStyle != 'red') {
    context.strokeStyle = 'red';
  }
  sendColorChange();
}
function changeYellow() {
  if (context.strokeStyle != 'yellow') {
    context.strokeStyle = 'yellow';
  }
  sendColorChange();
}
function changeGreen() {
  if (context.strokeStyle != 'green') {
    context.strokeStyle = 'green';
  }
  sendColorChange();
}
function changeBlue() {
  if (context.strokeStyle != 'blue') {
    context.strokeStyle = 'blue';
  }
  sendColorChange();
}
function changePurple() {
  if (context.strokeStyle != 'purple') {
    context.strokeStyle = 'purple';
  }
  sendColorChange();
}

function sendColorChange() {
  let previous_color = colorHistory.length ? colorHistory[colorHistory.length - 1] : '#000000'
  console.log(previous_color, context.strokeStyle);
  websocket.send(`C${t()},${context.strokeStyle},${previous_color}`);
}

for (const ev of ["touchstart", "mousedown"]) {
  canvas_touch.addEventListener(ev, function (e) {
    let pressure = 0.1;
    let x, y;
    if (e.touches && e.touches[0] && typeof e.touches[0]["force"] !== "undefined") {
      if (e.touches[0]["force"] > 0) {
        pressure = e.touches[0]["force"]
      }
      x = e.touches[0].pageX * 2
      y = e.touches[0].pageY * 2
    } else {
      pressure = 1.0
      x = e.pageX * 2
      y = e.pageY * 2
    }

    isMousedown = true

    // lineWidth = Math.log(pressure + 1) * 40
    linewidth = 1;
    context.lineWidth = lineWidth// pressure * 50;

    points.push({ x, y, lineWidth })
    const xy = convert_xy_to_normalized(x, y)
    websocket.send("M" + t() + "," + xy[0] + "," + xy[1]);
    drawOnCanvas(points)
  })
}

for (const ev of ['touchmove', 'mousemove']) {
  canvas_touch.addEventListener(ev, function (e) {
    if (!isMousedown) return
    e.preventDefault()

    let pressure = 0.1
    let x, y
    if (e.touches && e.touches[0] && typeof e.touches[0]["force"] !== "undefined") {
      if (e.touches[0]["force"] > 0) {
        pressure = e.touches[0]["force"]
      }
      x = e.touches[0].pageX * 2
      y = e.touches[0].pageY * 2
    } else {
      // pressure = 1.0
      pressure = 0.1
      x = e.pageX * 2
      y = e.pageY * 2
    }

    // smoothen line width
    // lineWidth = (Math.log(pressure + 1) * 40 * 0.2 + lineWidth * 0.8)
    lineWidth = 1;
    points.push({ x, y, lineWidth })
    const xy = convert_xy_to_normalized(x, y)
    websocket.send("L" + t() + "," + xy[0] + "," + xy[1])

    drawOnCanvas(points);

    requestIdleCallback(() => {
      $force.textContent = 'force = ' + pressure

      const touch = e.touches ? e.touches[0] : null
      if (touch) {
        $touches.innerHTML = `
          touchType = ${touch.touchType} ${touch.touchType === 'direct' ? '👆' : '✍️'} <br/>
          radiusX = ${touch.radiusX} <br/>
          radiusY = ${touch.radiusY} <br/>
          rotationAngle = ${touch.rotationAngle} <br/>
          altitudeAngle = ${touch.altitudeAngle} <br/>
          azimuthAngle = ${touch.azimuthAngle} <br/>
        `
      }
    })
  })
}

for (const ev of ['touchend', 'touchleave', 'mouseup']) {
  canvas_touch.addEventListener(ev, function (e) {
    let pressure = 0.1;
    let x, y;

    if (e.touches && e.touches[0] && typeof e.touches[0]["force"] !== "undefined") {
      if (e.touches[0]["force"] > 0) {
        pressure = e.touches[0]["force"]
      }
      x = e.touches[0].pageX * 2
      y = e.touches[0].pageY * 2
    } else {
      pressure = 1.0
      x = e.pageX * 2
      y = e.pageY * 2
    }

    isMousedown = false

    requestIdleCallback(function () {
      strokeHistory.push([...points]);
      colorHistory.push(context.strokeStyle);
      points = [];
    })
    const xy = convert_xy_to_normalized(x, y);
    websocket.send("U" + t() + "," + xy[0] + "," + xy[1])

    lineWidth = 0
  })
};
