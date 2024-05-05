// Dash effect on the landing page
function setupDashEffect() {
    // Check for the existence of the canvas element to avoid errors
    const canvas = document.getElementById('dashes-canvas');
    if (!canvas) return;

    // Create the dashes
    const dashes = [];
    const dashCount = 1000; // Adjust as necessary
    const dashLength = 7; // Length of each dash
    const ctx = canvas.getContext('2d');
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;

    // Populate dashes array with random positions and angles
    for (let i = 0; i < dashCount; i++) {
        const angle = Math.random() * 2 * Math.PI;
        dashes.push({ x: Math.random() * canvas.width, y: Math.random() * canvas.height, angle, length: dashLength });
    }

    // Function to draw dashes on the canvas
    function drawDashes() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
        ctx.lineWidth = 2;

        dashes.forEach((dash) => {
            const endX = dash.x + dash.length * Math.cos(dash.angle);
            const endY = dash.y + dash.length * Math.sin(dash.angle);

            ctx.beginPath();
            ctx.moveTo(dash.x, dash.y);
            ctx.lineTo(endX, endY);
            ctx.stroke();
        });
    }

    // Update canvas on mouse movement
    document.addEventListener('mousemove', (event) => {
        const mouseX = event.pageX;
        const mouseY = event.pageY;

        // Adjust dash positions and angles based on the mouse coordinates
        dashes.forEach((dash) => {
            const dx = dash.x - mouseX;
            const dy = dash.y - mouseY;
            const distance = Math.sqrt(dx * dx + dy * dy);
            const angleToMouse = Math.atan2(dy, dx);

            // Rotate dashes away if mouse is close
            if (distance < 250) {
                dash.angle = angleToMouse + Math.PI;
            }
        });

        drawDashes();
    });

    // Initial drawing
    drawDashes();

    // Adjust canvas size on window resize
    window.addEventListener('resize', () => {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
        drawDashes();
    });
}

// Intersection Observer for project page scrolling effect
function setupProjectScrollEffect() {
    // Ensure that we're on the projects page by checking if any projects exist
    const projects = document.querySelectorAll('.project');
    if (projects.length === 0) return;

    const observer = new IntersectionObserver((entries) => {
        entries.forEach((entry) => {
            if (entry.isIntersecting) {
                entry.target.classList.add('active');
            } else {
                entry.target.classList.remove('active');
            }
        });
    }, {
        threshold: 0.5, // Adjust the threshold to control when the effect is triggered
    });

    projects.forEach((project) => observer.observe(project));
}

// Initialize both effects based on page content
document.addEventListener('DOMContentLoaded', () => {
    setupDashEffect(); // Runs only if a canvas is detected
    setupProjectScrollEffect(); // Runs only if projects are detected
});
