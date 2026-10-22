// Plot component helper (T10): reports the rendered width of a plot so that the SVG viewBox matches
// screen pixels and tick labels keep their size. Only a changed width is sent. ResizeObserver already
// delivers at most once per frame; requestAnimationFrame is not used because it does not run in a
// hidden tab, which would leave the plot at its default width.
window.robotArmPlot = (function () {
    const observers = new WeakMap();

    function observe(element, dotNetRef) {
        if (!element || observers.has(element)) return;
        let lastWidth = -1;
        const report = () => {
            const width = Math.round(element.clientWidth);
            if (width > 0 && width !== lastWidth) {
                lastWidth = width;
                dotNetRef.invokeMethodAsync('SetWidth', width);
            }
        };
        const ro = new ResizeObserver(report);
        ro.observe(element);
        observers.set(element, ro);
        report();
    }

    function unobserve(element) {
        const ro = element && observers.get(element);
        if (ro) {
            ro.disconnect();
            observers.delete(element);
        }
    }

    return { observe, unobserve };
})();
