"""
MkDocs macros for moteus documentation
"""

def define_env(env):
    """
    Define custom macros and filters for MkDocs
    """

    @env.macro
    def youtube(video_id, max_width=600):
        """
        Embed a YouTube video with responsive sizing

        Usage in markdown:
            {{ youtube("dQw4w9WgXcQ") }}
            {{ youtube("dQw4w9WgXcQ", 800) }}

        Args:
            video_id: YouTube video ID
            max_width: Maximum width in pixels (default: 600)
        """
        return f'''<div style="max-width: {max_width}px; width: 100%; margin: 0 auto;">
  <iframe style="width: 100%; aspect-ratio: 16/9; border: none;"
          src="https://www.youtube.com/embed/{video_id}"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
          allowfullscreen>
  </iframe>
</div>'''
