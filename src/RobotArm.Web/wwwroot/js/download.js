// File download helper (T10). Blazor WebAssembly has no file system, so the bytes are wrapped in a Blob,
// given an object URL and saved through a temporary <a download> link. Used for the CSV log and the
// scenario JSON. Nothing leaves the browser.
window.robotArmFiles = {
    download: function (fileName, mimeType, bytes) {
        const blob = new Blob([bytes], { type: mimeType });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = fileName;
        a.style.display = 'none';
        document.body.appendChild(a);
        a.click();
        a.remove();
        // Revoke later: some browsers start reading the URL only after click() returns.
        setTimeout(() => URL.revokeObjectURL(url), 10000);
    }
};
