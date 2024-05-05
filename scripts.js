// scripts.js
// scripts.js
document.addEventListener('mousemove', function (event) {
    // Create a div element to represent each "dot" in the trail
    const trailDot = document.createElement('div');
    trailDot.className = 'trail-dot';
    trailDot.style.left = `${event.pageX}px`;
    trailDot.style.top = `${event.pageY}px`;
    document.body.appendChild(trailDot);

    // Remove the dot after some time to simulate fading away
    setTimeout(() => {
        trailDot.remove();
    }, 500); // Adjust the timing to control the trail duration
});


// scripts.js
document.addEventListener('DOMContentLoaded', function () {
    const projects = document.querySelectorAll('.project');

    const observer = new IntersectionObserver(entries => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.classList.add('active');
            } else {
                entry.target.classList.remove('active');
            }
        });
    }, {
        threshold: 0.5 // Adjust the threshold for when to trigger the effect
    });

    projects.forEach(project => observer.observe(project));
});
