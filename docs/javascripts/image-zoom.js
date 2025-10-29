// Image zoom functionality using iv-viewer
(function() {
  'use strict';

  // Wait for iv-viewer library to load
  function initImageZoom() {
    // Check if ImageViewer is available
    if (typeof ImageViewer === 'undefined') {
      console.error('ImageViewer not found. Make sure iv-viewer.min.js is loaded.');
      return;
    }

    var FullScreenViewer = ImageViewer.FullScreenViewer;

    // Find all images that should be zoomable
    // Only apply to SVG images on the reference/pinouts page
    var zoomableImages = [];

    // Check if we're on the pinouts page
    if (window.location.pathname.includes('/reference/pinouts')) {
      // Select all SVG images on this page
      zoomableImages = document.querySelectorAll('img[src$=".svg"]');
    }

    zoomableImages.forEach(function(img) {
      // Skip if already processed
      if (img.dataset.zoomEnabled) {
        return;
      }

      // Mark as processed
      img.dataset.zoomEnabled = 'true';

      // Make image clickable
      img.style.cursor = 'zoom-in';
      img.title = 'Click to zoom';

      // Add click handler
      img.addEventListener('click', function(e) {
        e.preventDefault();
        e.stopPropagation();

        // Create fullscreen viewer
        const viewer = new FullScreenViewer({
          maxZoom: 500,
          zoomValue: 100,
          snapView: true,
          refreshOnResize: true
        });

        // Add ESC key handler to close viewer
        var escKeyHandler = function(e) {
          if (e.key === 'Escape' || e.keyCode === 27) {
            viewer.hide();
            document.removeEventListener('keydown', escKeyHandler);
          }
        };
        document.addEventListener('keydown', escKeyHandler);

        // Also clean up ESC handler when viewer is closed by other means (click on X button)
        var originalHide = viewer.hide.bind(viewer);
        viewer.hide = function() {
          document.removeEventListener('keydown', escKeyHandler);
          originalHide();
        };

        // Show the image
        viewer.show(img.src, img.src);
      });
    });
  }

  // Initialize when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initImageZoom);
  } else {
    initImageZoom();
  }

  // Re-initialize when Material for MkDocs navigates to a new page
  // (Material uses instant navigation which doesn't trigger full page loads)
  document.addEventListener('DOMContentLoaded', function() {
    // Try to hook into Material's document$ observable if available
    if (typeof document$ !== 'undefined') {
      document$.subscribe(function() {
        setTimeout(initImageZoom, 100);
      });
    }
  });

  // Fallback: detect page navigation by watching pathname changes
  let lastPathname = window.location.pathname;
  setInterval(function() {
    if (window.location.pathname !== lastPathname) {
      lastPathname = window.location.pathname;
      setTimeout(initImageZoom, 100);
    }
  }, 500);
})();
