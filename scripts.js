// scripts.js
document.addEventListener('mousemove', function (event) {
    const glow = document.createElement('div');
    glow.className = 'glow';
    glow.style.left = `${event.pageX}px`;
    glow.style.top = `${event.pageY}px`;
    document.body.appendChild(glow);

    setTimeout(() => {
        glow.remove();
    }, 400); 
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
