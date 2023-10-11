// tiempo máximo de la simulación
let TIME_LIMIT = 21;


const FULL_DASH_ARRAY = 283;
const WARNING_THRESHOLD = 188;
const ALERT_THRESHOLD = 94;

const COLOR_CODES = {
    info: {
        color: "green"
    },
    warning: {
        color: "orange",
        threshold: WARNING_THRESHOLD
    },
    alert: {
        color: "red",
        threshold: ALERT_THRESHOLD
    }
};

let timeLeft = TIME_LIMIT;
let timePassed = 0;
let timerInterval = null;
let remainingPathColor = COLOR_CODES.info.color;

document.getElementById("app").innerHTML = `
<div class="base-timer">
  <svg class="base-timer__svg" viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
    <g class="base-timer__circle">
      <circle class="base-timer__path-elapsed" cx="50" cy="50" r="45"></circle>
      <path
        id="base-timer-path-remaining"
        stroke-dasharray="283"
        class="base-timer__path-remaining ${remainingPathColor}"
        d="
          M 50, 50
          m -45, 0
          a 45,45 0 1,0 90,0
          a 45,45 0 1,0 -90,0
        "
      ></path>
    </g>
  </svg>
  <span id="base-timer-label" class="base-timer__label">${formatTime(timeLeft)}</span>
</div>
`;



function onTimesUp() {
    clearInterval(timerInterval);
}

// startTimer();
function startTimer() {
    timerInterval = setInterval(() => {
        timePassed = timePassed += 1;
        timeLeft = TIME_LIMIT - timePassed;
        document.getElementById("base-timer-label").innerHTML = formatTime(timeLeft);
        setCircleDasharray();
        setRemainingPathColor(timeLeft);

        if (timeLeft === 0) {
            onTimesUp();
        }
    }, 1000);
}

function set_time(time, lim_sup) {
    TIME_LIMIT = lim_sup;
    timeLeft = constrain(time, 0, TIME_LIMIT);

    if (timeLeft <= 0)
        timeLeft = 0

    document.getElementById("base-timer-label").innerHTML = formatTime(timeLeft);
    setCircleDasharray();
    setRemainingPathColor(timeLeft);


}

function formatTime(time) {
    const minutes = Math.floor(time / 60);
    let seconds = time % 60;

    if (seconds < 10) {
        seconds = `0${seconds}`;
    }

    return `${minutes}:${seconds}`;
}

function setRemainingPathColor(timeLeft) {
    const { alert, warning, info } = COLOR_CODES;

    let timer = document.getElementById("base-timer-path-remaining");
    timer.classList.remove("green", "orange", "red");

    let time = map_val(timeLeft, 0, TIME_LIMIT, 0, FULL_DASH_ARRAY - 1);
    if (time > warning.threshold) {
        timer.classList.add(info.color);

    } else if (warning.threshold >= time && time > alert.threshold) {
        timer.classList.add(warning.color);

    } else if (time <= alert.threshold) {
        timer.classList.add(alert.color);
    }
}

function calculateTimeFraction() {
    const rawTimeFraction = timeLeft / TIME_LIMIT;
    return rawTimeFraction - (1 / TIME_LIMIT) * (1 - rawTimeFraction);
}

function setCircleDasharray() {
    const circleDasharray = `${(calculateTimeFraction() * FULL_DASH_ARRAY).toFixed(0)} 283`;
    document
        .getElementById("base-timer-path-remaining")
        .setAttribute("stroke-dasharray", circleDasharray);
}

let cc = 0


let actual_seg;
function clock_subcribe() {
    rosnodejs.nh.subscribe("/clock", "rosgraph_msgs/Clock", (msg) => {
        let seg = msg.clock.secs;

        if (actual_seg != seg) {
            actual_seg = seg;

            // se verifica el estado del robot
            control_robot().then((res) => {
                // se encuentra manejando
                if (res.is_driving) {
                    // console.log(res.max_time - actual_seg, res.max_time);
                    set_time(res.max_time - actual_seg, res.max_time);
                }
            });
        }
    });
}

clock_subcribe();