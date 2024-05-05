// scripts.js
document.addEventListener('mousemove', function (event) {
    const glow = document.createElement('div');
    glow.className = 'glow';
    glow.style.left = `${event.pageX}px`;
    glow.style.top = `${event.pageY}px`;
    document.body.appendChild(glow);

    setTimeout(() => {
        glow.remove();
    }, 100); // Removes after 1 second
});

