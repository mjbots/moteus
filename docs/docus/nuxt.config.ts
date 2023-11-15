import { createResolver } from '@nuxt/kit'
const { resolve } = createResolver(import.meta.url)

export default defineNuxtConfig({
  // https://github.com/nuxt-themes/docus
  extends: '@nuxt-themes/docus',
  devtools: { enabled: false },
  css: ['~/assets/css/root.css'],

  modules: [
    // Allow images and media content within the content directory
    'nuxt-content-assets',
  ],

  // Nuxt Content module (Markdown) settings
  content: {
    // Provide alternate sources directory
    sources: {
      content: {
        driver: 'fs',
        base: resolve(__dirname, './../content'),
        experimental: {
          search: true
        }
      }
    },
    // markdown code highlight syntax's 
    highlight: {
      preload: ['diff', 'json', 'js', 'ts', 'css', 'shell', 'html', 'md', 'yaml', 'py', 'sh']
    },
  },

  // Fix for bug https://github.com/davestewart/nuxt-content-assets/issues/49
  hooks: {
    close: (nuxt) => {
      if (!nuxt.options._prepare)
        process.exit()
    }
  },

  // Settings for Github Pages
  app: {
    baseURL: process.env.BASE_URL || '/',
  }

})
