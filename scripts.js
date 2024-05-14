const canvas = document.getElementById('manipulator-canvas');
const ctx = canvas.getContext('2d');
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

// Adjusted base coordinates to be closer to the canvas center
let baseX = canvas.width / 3, baseY = canvas.height / 2, baseZ = 0;
let armLengths = [100, 100, 100]; // Keep these lengths reasonable for visibility
let jointAngles = [0, 0, 0]; // Initialize joint angles at zero
let maxReach = armLengths.reduce((a, b) => a + b, 0);

console.log("Base coordinates:", baseX, baseY, baseZ);
console.log("Canvas dimensions:", canvas.width, canvas.height);
console.log("Arm lengths:", armLengths);
console.log("Initial Joint Angles:", jointAngles);

// PID control gains
let Kp = 0.0, Ki = 0.0, Kd = 0.0;
let integralError = [0, 0, 0], previousError = [0, 0, 0];

const kpSlider = document.getElementById('kp-slider');
const kdSlider = document.getElementById('kd-slider');
const kpValueLabel = document.getElementById('kp-value');
const kdValueLabel = document.getElementById('kd-value');

kpSlider.addEventListener('input', () => {
    Kp = parseFloat(kpSlider.value);
    kpValueLabel.textContent = Kp.toFixed(1);
    console.log("Updated Kp:", Kp);
});

kdSlider.addEventListener('input', () => {
    Kd = parseFloat(kdSlider.value);
    kdValueLabel.textContent = Kd.toFixed(1);
    console.log("Updated Kd:", Kd);
});

// Clamp target position within max reach
function clampTargetPosition(x, y, z) {
    const dx = x - baseX;
    const dy = y - baseY;
    const dz = z - baseZ;
    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (distance > maxReach) {
        x = baseX + (dx / distance) * maxReach;
        y = baseY + (dy / distance) * maxReach;
        z = baseZ + (dz / distance) * maxReach;
    }
    console.log("Clamped target position:", x, y, z);
    return [x, y, z];
}

// Forward kinematics function to compute joint positions in 3D
function fkine3D(angles) {
    let cumulativeAngle = 0;
    let currentX = baseX, currentY = baseY, currentZ = baseZ;
    const positions = [{ x: currentX, y: currentY, z: currentZ }];

    for (let i = 0; i < armLengths.length; i++) {
        cumulativeAngle += angles[i]; // Sum angles

        currentX += armLengths[i] * Math.cos(cumulativeAngle); // Adjust X position
        currentY += armLengths[i] * Math.sin(cumulativeAngle); // Adjust Y position

        positions.push({ x: currentX, y: currentY, z: currentZ });
    }

    console.log("Computed positions (3D):", positions);
    return positions;
}

const focalLength = 500; // Adjust focal length for better visibility

// Project 3D point to 2D canvas
function project3Dto2D(point3D) {
    const scale = focalLength / (focalLength + point3D.z);
    const x2D = point3D.x * scale + canvas.width / 2; // Center horizontally
    const y2D = point3D.y * scale + canvas.height / 2; // Center vertically
    return { x: x2D, y: y2D };
}


// Update joint angles
function updateAngles3D(targetX, targetY, targetZ) {
    [targetX, targetY, targetZ] = clampTargetPosition(targetX, targetY, targetZ);

    const positions = fkine3D(jointAngles);
    const endEffector = positions[positions.length - 1];
    const error = [targetX - endEffector.x, targetY - endEffector.y, targetZ - endEffector.z];

    console.log("Target position:", targetX, targetY, targetZ);
    console.log("End effector position:", endEffector);
    console.log("Position error:", error);

    integralError[0] += error[0]; integralError[1] += error[1]; integralError[2] += error[2];
    const derivativeError = [
        error[0] - previousError[0],
        error[1] - previousError[1],
        error[2] - previousError[2]
    ];
    previousError = [...error];

    const dp = [
        Kp * error[0] + Ki * integralError[0] + Kd * derivativeError[0],
        Kp * error[1] + Ki * integralError[1] + Kd * derivativeError[1],
        Kp * error[2] + Ki * integralError[2] + Kd * derivativeError[2]
    ];

    // Jacobian and pseudoinverse calculation for 3D
    const J = jacobian3D(jointAngles); // Update to 3D Jacobian
    const J_pseudo = pseudoJacobian3D(J); // Update to 3D pseudoinverse

    const dTheta = [
        J_pseudo[0][0] * dp[0] + J_pseudo[0][1] * dp[1] + J_pseudo[0][2] * dp[2],
        J_pseudo[1][0] * dp[0] + J_pseudo[1][1] * dp[1] + J_pseudo[1][2] * dp[2],
        J_pseudo[2][0] * dp[0] + J_pseudo[2][1] * dp[1] + J_pseudo[2][2] * dp[2]
    ];

    const maxChange = 0.5;
    jointAngles = jointAngles.map((angle, i) => angle + clamp(dTheta[i] * 0.1, -maxChange, maxChange));
    console.log("Updated Joint Angles:", jointAngles);
}

// Draw circles at each joint position for easier debugging
function drawCircle(x, y, radius, color) {
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.stroke();
}

// Draw segments between joints
function drawManipulator3D(positions) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    for (let i = 0; i < positions.length; i++) {
        const projectedPos = project3Dto2D(positions[i]);

        // Draw a circle at each joint position to visualize each joint
        drawCircle(projectedPos.x, projectedPos.y, 5, 'red');

        if (i < positions.length - 1) {
            const nextPos = project3Dto2D(positions[i + 1]);

            console.log("Drawing segment:", projectedPos, nextPos);

            ctx.beginPath();
            ctx.moveTo(projectedPos.x, projectedPos.y);
            ctx.lineTo(nextPos.x, nextPos.y);
            ctx.strokeStyle = '#007bff';
            ctx.lineWidth = 5;
            ctx.stroke();
        }
    }
}


function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

// 3D Jacobian calculation
function jacobian3D(angles) {
    const l1 = armLengths[0], l2 = armLengths[1], l3 = armLengths[2];
    const q1 = angles[0], q2 = angles[1], q3 = angles[2];

    const sinQ1 = Math.sin(q1), cosQ1 = Math.cos(q1);
    const sinQ12 = Math.sin(q1 + q2), cosQ12 = Math.cos(q1 + q2);
    const sinQ123 = Math.sin(q1 + q2 + q3), cosQ123 = Math.cos(q1 + q2 + q3);

    const J = [
        [
            -l1 * sinQ1 - l2 * sinQ12 - l3 * sinQ123, // ∂x/∂q1
            -l2 * sinQ12 - l3 * sinQ123,             // ∂x/∂q2
            -l3 * sinQ123                            // ∂x/∂q3
        ],
        [
            l1 * cosQ1 + l2 * cosQ12 + l3 * cosQ123, // ∂y/∂q1
            l2 * cosQ12 + l3 * cosQ123,              // ∂y/∂q2
            l3 * cosQ123                             // ∂y/∂q3
        ],
        [0, 0, 0] // Simplified approximation for Z-axis
    ];

    console.log("Computed Jacobian:", J);
    return J;
}

// 3D Pseudoinverse Jacobian calculation
function pseudoJacobian3D(J) {
    const JT = [
        [J[0][0], J[1][0], J[2][0]], // J^T row 1
        [J[0][1], J[1][1], J[2][1]], // J^T row 2
        [J[0][2], J[1][2], J[2][2]]  // J^T row 3
    ];

    const JJt = [
        [
            J[0][0] * J[0][0] + J[0][1] * J[0][1] + J[0][2] * J[0][2], // First row first col
            J[0][0] * J[1][0] + J[0][1] * J[1][1] + J[0][2] * J[1][2], // First row second col
            J[0][0] * J[2][0] + J[0][1] * J[2][1] + J[0][2] * J[2][2]  // First row third col
        ],
        [
            J[1][0] * J[0][0] + J[1][1] * J[0][1] + J[1][2] * J[0][2],
            J[1][0] * J[1][0] + J[1][1] * J[1][1] + J[1][2] * J[1][2],
            J[1][0] * J[2][0] + J[1][1] * J[2][1] + J[1][2] * J[2][2]
        ],
        [
            J[2][0] * J[0][0] + J[2][1] * J[0][1] + J[2][2] * J[0][2],
            J[2][0] * J[1][0] + J[2][1] * J[1][1] + J[2][2] * J[1][2],
            J[2][0] * J[2][0] + J[2][1] * J[2][1] + J[2][2] * J[2][2]
        ]
    ];

    // Add damping factor to improve numerical stability
    const dampingFactor = 0.005;
    for (let i = 0; i < 3; i++) {
        JJt[i][i] += dampingFactor;
    }

    // Compute inverse of JJt
    const detJJt = JJt[0][0] * (JJt[1][1] * JJt[2][2] - JJt[1][2] * JJt[2][1]) -
        JJt[0][1] * (JJt[1][0] * JJt[2][2] - JJt[1][2] * JJt[2][0]) +
        JJt[0][2] * (JJt[1][0] * JJt[2][1] - JJt[1][1] * JJt[2][0]);

    if (Math.abs(detJJt) < 1e-5) {
        console.log("Singular matrix detected.");
        return [[0, 0, 0], [0, 0, 0], [0, 0, 0]]; // Zero out to avoid numerical instability
    }

    const JJtInv = [
        [
            (JJt[1][1] * JJt[2][2] - JJt[1][2] * JJt[2][1]) / detJJt,
            (JJt[0][2] * JJt[2][1] - JJt[0][1] * JJt[2][2]) / detJJt,
            (JJt[0][1] * JJt[1][2] - JJt[0][2] * JJt[1][1]) / detJJt
        ],
        [
            (JJt[1][2] * JJt[2][0] - JJt[1][0] * JJt[2][2]) / detJJt,
            (JJt[0][0] * JJt[2][2] - JJt[0][2] * JJt[2][0]) / detJJt,
            (JJt[0][2] * JJt[1][0] - JJt[0][0] * JJt[1][2]) / detJJt
        ],
        [
            (JJt[1][0] * JJt[2][1] - JJt[1][1] * JJt[2][0]) / detJJt,
            (JJt[0][1] * JJt[2][0] - JJt[0][0] * JJt[2][1]) / detJJt,
            (JJt[0][0] * JJt[1][1] - JJt[0][1] * JJt[1][0]) / detJJt
        ]
    ];

    // Pseudoinverse calculation
    const J_pseudo = [
        [
            JT[0][0] * JJtInv[0][0] + JT[0][1] * JJtInv[1][0] + JT[0][2] * JJtInv[2][0],
            JT[0][0] * JJtInv[0][1] + JT[0][1] * JJtInv[1][1] + JT[0][2] * JJtInv[2][1],
            JT[0][0] * JJtInv[0][2] + JT[0][1] * JJtInv[1][2] + JT[0][2] * JJtInv[2][2]
        ],
        [
            JT[1][0] * JJtInv[0][0] + JT[1][1] * JJtInv[1][0] + JT[1][2] * JJtInv[2][0],
            JT[1][0] * JJtInv[0][1] + JT[1][1] * JJtInv[1][1] + JT[1][2] * JJtInv[2][1],
            JT[1][0] * JJtInv[0][2] + JT[1][1] * JJtInv[1][2] + JT[1][2] * JJtInv[2][2]
        ],
        [
            JT[2][0] * JJtInv[0][0] + JT[2][1] * JJtInv[1][0] + JT[2][2] * JJtInv[2][0],
            JT[2][0] * JJtInv[0][1] + JT[2][1] * JJtInv[1][1] + JT[2][2] * JJtInv[2][1],
            JT[2][0] * JJtInv[0][2] + JT[2][1] * JJtInv[1][2] + JT[2][2] * JJtInv[2][2]
        ]
    ];

    console.log("Computed pseudoinverse:", J_pseudo);
    return J_pseudo;
}

// Adjust the target position based on touch or mouse events
function handleMouse(event) {
    const targetZ = 100; // For simplicity, keep the z-plane static, or change based on input
    console.log("Mouse event:", event.clientX, event.clientY, targetZ);
    updateAngles3D(event.clientX, event.clientY, targetZ);
    const positions = fkine3D(jointAngles);
    drawManipulator3D(positions);
}

// Event listeners for user input
canvas.addEventListener('mousemove', handleMouse);
canvas.addEventListener('touchstart', handleMouse);
canvas.addEventListener('touchmove', handleMouse);

// Initial draw
const initialPositions = fkine3D(jointAngles);
drawManipulator3D(initialPositions);