// Make ToC collapsible at h2 level (class boundaries)
(function() {
  // Track collapsed state
  const collapsedState = new Map();

  // Pages where collapsible ToC should be enabled
  const enabledPages = [
    '/reference/python/',
  ];

  function isCollapsibleTocEnabled() {
    const path = window.location.pathname;
    return enabledPages.some(function(enabledPath) {
      return path.includes(enabledPath);
    });
  }

  function setupCollapsibleToc() {
    // Only run on enabled pages
    if (!isCollapsibleTocEnabled()) {
      return;
    }

    // Find the visible ToC only (not the mobile duplicate)
    const toc = document.querySelector(".md-sidebar--secondary .md-nav--secondary");
    if (!toc) {
      return;
    }

    // Find all h2 level ToC items (class names like "moteus.Controller")
    const tocItems = toc.querySelectorAll(":scope > .md-nav__list > .md-nav__item");

    tocItems.forEach(function(item) {
      const link = item.querySelector(":scope > .md-nav__link");
      const subNav = item.querySelector(":scope > .md-nav");

      if (link && subNav && !link.dataset.tocCollapsible) {
        // Mark as processed
        link.dataset.tocCollapsible = "true";

        // Store the original href
        const originalHref = link.getAttribute("href");
        const itemKey = originalHref || link.textContent.trim();

        // Store references for event delegation
        item.dataset.tocItemKey = itemKey;
        item.dataset.tocHref = originalHref;

        // Wrap the text in a span for clicking to navigate
        const textContent = link.childNodes[0];
        const textSpan = document.createElement("span");
        textSpan.textContent = textContent.textContent;
        textSpan.style.cssText = "flex: 1; cursor: pointer;";
        textSpan.classList.add("toc-collapse-text");
        link.replaceChild(textSpan, textContent);

        // Make link a flex container and prevent default behavior
        link.style.cssText = "display: flex !important; align-items: center; cursor: default; position: relative;";
        link.removeAttribute("href"); // Remove href to prevent Material from handling it
        link.dataset.originalHref = originalHref;

        // Add collapse indicator as a button
        const indicator = document.createElement("span");
        indicator.innerHTML = "▼";
        indicator.style.cssText = "margin-left: auto; transition: transform 0.2s; font-size: 0.8em; cursor: pointer; padding: 0 0.5em;";
        indicator.classList.add("toc-collapse-indicator");
        link.appendChild(indicator);

        // Restore previous state or collapse by default
        const isCollapsed = collapsedState.has(itemKey) ? collapsedState.get(itemKey) : true;

        if (isCollapsed) {
          subNav.style.display = "none";
          indicator.style.transform = "rotate(-90deg)";
        } else {
          subNav.style.display = "block";
          indicator.style.transform = "rotate(0deg)";
        }
      }
    });
  }

  // Use event delegation on sidebar
  document.addEventListener("click", function(e) {
    // Check if click is on indicator
    if (e.target.classList.contains("toc-collapse-indicator")) {
      e.preventDefault();
      e.stopPropagation();

      const link = e.target.closest(".md-nav__link");
      const item = link?.closest(".md-nav__item");
      const subNav = item?.querySelector(":scope > .md-nav");
      const itemKey = item?.dataset.tocItemKey;

      if (subNav && itemKey) {
        if (subNav.style.display === "none") {
          subNav.style.display = "block";
          e.target.style.transform = "rotate(0deg)";
          collapsedState.set(itemKey, false);
        } else {
          subNav.style.display = "none";
          e.target.style.transform = "rotate(-90deg)";
          collapsedState.set(itemKey, true);
        }
      }
      return;
    }

    // Check if click is on text
    if (e.target.classList.contains("toc-collapse-text")) {
      e.preventDefault();
      e.stopPropagation();

      const link = e.target.closest(".md-nav__link");
      const originalHref = link?.dataset.originalHref;

      if (originalHref) {
        // Just update the hash - let the browser handle the scrolling
        window.location.hash = originalHref.replace(/^[^#]*/, '');
      }
      return;
    }

    // Fallback: check if clicked on a modified ToC link
    const link = e.target.closest(".md-nav__link[data-original-href]");
    if (link && link.dataset.tocCollapsible === "true") {
      e.preventDefault();
      e.stopPropagation();

      const originalHref = link.dataset.originalHref;
      if (originalHref) {
        window.location.hash = originalHref.replace(/^[^#]*/, '');
      }
      return;
    }
  }, true); // Use capture phase

  // Initial setup
  document.addEventListener("DOMContentLoaded", function() {
    setupCollapsibleToc();
  });

  // Listen for Material's instant navigation completing
  document.addEventListener("DOMContentLoaded", function() {
    // Try to hook into Material's document$ observable if available
    if (typeof document$ !== 'undefined') {
      document$.subscribe(function() {
        setTimeout(setupCollapsibleToc, 200);
      });
    }
  });

  // Fallback: detect page navigation by watching pathname changes
  let lastPathname = window.location.pathname;
  setInterval(function() {
    if (window.location.pathname !== lastPathname) {
      lastPathname = window.location.pathname;
      setTimeout(setupCollapsibleToc, 200);
    }
  }, 500);

  // Re-apply when Material updates the ToC - watch the sidebar container
  const observer = new MutationObserver(function(mutations) {
    let hasRelevantChanges = false;
    mutations.forEach(function(mutation) {
      if (mutation.addedNodes.length > 0) {
        hasRelevantChanges = true;
      }
    });
    if (hasRelevantChanges) {
      setTimeout(setupCollapsibleToc, 100); // Small delay to let Material finish
    }
  });

  // Observe the entire sidebar for changes
  document.addEventListener("DOMContentLoaded", function() {
    const sidebar = document.querySelector(".md-sidebar--secondary");
    if (sidebar) {
      observer.observe(sidebar, { childList: true, subtree: true });
    }
  });

  // Re-apply collapsed state when hash changes (Material re-renders ToC)
  window.addEventListener("hashchange", function() {
    setTimeout(function() {
      setupCollapsibleToc();
    }, 150); // Wait for Material to finish rendering
  });
})();
