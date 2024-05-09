const canvas = document.getElementById('manipulator-canvas');
const ctx = canvas.getContext('2d');
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

const titleContainer = document.querySelector(".title-container");
const titleContainerBounds = titleContainer.getBoundingClientRect();
let baseX, baseY;
let armLengths, jointAngles;
let maxReach;

// Detect if the device is mobile or desktop
if (window.innerWidth <= 768) {
    // Mobile settings
    baseX = titleContainerBounds.left - 10;
    baseY = titleContainerBounds.bottom - 20;
    armLengths = [100, 100];
    jointAngles = [0, 0]; 
} else {
    // Desktop settings
    baseX = titleContainerBounds.left - 65;
    baseY = titleContainerBounds.bottom + 20;
    armLengths = [300, 300]; 
    jointAngles = [0, 0]; 
}

maxReach = armLengths.reduce((a, b) => a + b, 0);

// gains
let Kp = 0.0, Ki = 0.0, Kd = 0.0;
let integralError = [0, 0], previousError = [0, 0];

const kpSlider = document.getElementById('kp-slider');
const kdSlider = document.getElementById('kd-slider');
const kpValueLabel = document.getElementById('kp-value');
const kdValueLabel = document.getElementById('kd-value');

kpSlider.addEventListener('input', () => {
    Kp = parseFloat(kpSlider.value);
    kpValueLabel.textContent = Kp.toFixed(1);
});

kdSlider.addEventListener('input', () => {
    Kd = parseFloat(kdSlider.value);
    kdValueLabel.textContent = Kd.toFixed(1);
});

function clampTargetPosition(x, y) {
    const dx = x - baseX;
    const dy = y - baseY;
    const distance = Math.sqrt(dx * dx + dy * dy);

    if (distance > maxReach) {
        x = baseX + (dx / distance) * maxReach;
        y = baseY + (dy / distance) * maxReach;
    }
    return [x, y];
}

function fkine(angles) {
    let cumulativeAngle = 0;
    let currentX = baseX, currentY = baseY;
    const positions = [{ x: currentX, y: currentY }];

    for (let i = 0; i < armLengths.length; i++) {
        cumulativeAngle += angles[i];
        currentX += armLengths[i] * Math.cos(cumulativeAngle);
        currentY += armLengths[i] * Math.sin(cumulativeAngle);
        positions.push({ x: currentX, y: currentY });
    }
    return positions;
}

function jacobian(angles) {
    const J = [[0, 0], [0, 0]];
    const l1 = armLengths[0], l2 = armLengths[1];
    const q1 = angles[0], q2 = angles[1];
    const sinQ1 = Math.sin(q1), cosQ1 = Math.cos(q1);
    const sinQ12 = Math.sin(q1 + q2), cosQ12 = Math.cos(q1 + q2);

    J[0][0] = -l1 * sinQ1 - l2 * sinQ12;
    J[0][1] = -l2 * sinQ12;
    J[1][0] = l1 * cosQ1 + l2 * cosQ12;
    J[1][1] = l2 * cosQ12;

    return J;
}

// Damped pseudoinverse Jacobian, this function is used to calculate the joint angles based on the target position and ensures numerical stability
function pseudoJacobian(J) {
    const JT = [[], []];
    JT[0][0] = J[0][0]; JT[0][1] = J[1][0];
    JT[1][0] = J[0][1]; JT[1][1] = J[1][1];

    const JJt = [
        [J[0][0] * J[0][0] + J[0][1] * J[0][1], J[0][0] * J[1][0] + J[0][1] * J[1][1]],
        [J[1][0] * J[0][0] + J[1][1] * J[0][1], J[1][0] * J[1][0] + J[1][1] * J[1][1]]
    ];

    // Add damping factor to JJ^T to improve numerical stability
    const dampingFactor = 0.005;
    JJt[0][0] += dampingFactor;
    JJt[1][1] += dampingFactor;

    // Inverse of JJ^T
    const detJJt = JJt[0][0] * JJt[1][1] - JJt[0][1] * JJt[1][0];
    if (Math.abs(detJJt) < 1e-5) {
        console.log("Singular matrix detected.");
        return [[0, 0], [0, 0]]; // Zero out to avoid numerical instability
    }

    const JJtInv = [
        [JJt[1][1] / detJJt, -JJt[0][1] / detJJt],
        [-JJt[1][0] / detJJt, JJt[0][0] / detJJt]
    ];

    // Calculate the pseudoinverse
    return [
        [JT[0][0] * JJtInv[0][0] + JT[0][1] * JJtInv[1][0], JT[0][0] * JJtInv[0][1] + JT[0][1] * JJtInv[1][1]],
        [JT[1][0] * JJtInv[0][0] + JT[1][1] * JJtInv[1][0], JT[1][0] * JJtInv[0][1] + JT[1][1] * JJtInv[1][1]]
    ];
}

function updateAngles(targetX, targetY) {
    [targetX, targetY] = clampTargetPosition(targetX, targetY);

    const positions = fkine(jointAngles);
    const endEffector = positions[positions.length - 1];
    const error = [targetX - endEffector.x, targetY - endEffector.y];

    integralError[0] += error[0]; integralError[1] += error[1];
    const derivativeError = [error[0] - previousError[0], error[1] - previousError[1]];
    previousError = [...error];

    const dp = [
        Kp * error[0] + Ki * integralError[0] + Kd * derivativeError[0],
        Kp * error[1] + Ki * integralError[1] + Kd * derivativeError[1]
    ];

    const J = jacobian(jointAngles);
    const J_pseudo = pseudoJacobian(J);

    const dTheta = [
        J_pseudo[0][0] * dp[0] + J_pseudo[0][1] * dp[1],
        J_pseudo[1][0] * dp[0] + J_pseudo[1][1] * dp[1]
    ];

    const maxChange = 0.5; // Adjust the value to control the sensitivity
    jointAngles = jointAngles.map((angle, i) => angle + clamp(dTheta[i] * 0.1, -maxChange, maxChange));
}

// Draw the robotic manipulator
function drawManipulator(positions) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    for (let i = 0; i < positions.length - 1; i++) {
        ctx.beginPath();
        ctx.moveTo(positions[i].x, positions[i].y);
        ctx.lineTo(positions[i + 1].x, positions[i + 1].y);
        ctx.strokeStyle = '#007bff';
        ctx.lineWidth = 5;
        ctx.stroke();
    }
}

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

// Adjust the target position based on touch events
function handleTouch(event) {
    const touch = event.touches[0] || event.changedTouches[0];
    updateAngles(touch.clientX, touch.clientY);
    const positions = fkine(jointAngles);
    drawManipulator(positions);
}

// Event listeners for touch events
canvas.addEventListener('touchstart', handleTouch);
canvas.addEventListener('touchmove', handleTouch);


document.addEventListener('mousemove', (event) => {
    updateAngles(event.clientX, event.clientY);
    const positions = fkine(jointAngles);
    drawManipulator(positions);
});

const initialPositions = fkine(jointAngles);
drawManipulator(initialPositions);
