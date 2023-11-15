# Readme

This is the documentation for the moteus repo.

  * The content directory contains any markdown and image files.
  * The docus directory contains a nuxt docus static site generator for the documentation

## Static Site Generator

The static site generator uses the nuxt content module with the docus template
it also uses the nuxt-content-assets module so we can have media / images in the content directory

  * https://docus.dev/
  * https://github.com/davestewart/nuxt-content-assets

In order to use this I'd suggest installing the latest version of nodejs and pnpm

```sh
# Install the needed packages
cd docs/docus
pnpm install

# Generate the site to .output
pnpm generate

# Preview the site in a browser
pnpm preview
```

### Other commands

```sh
# It's also possible to run the site in dev mode to see live updates to markdown files
pnpm dev

# To check for any updates to package.json
pnpm upd
```
