export default defineAppConfig({
  docus: {
    title: 'moteus',
    description: 'moteus brushless servo.',
    image: 'https://user-images.githubusercontent.com/904724/185365452-87b7ca7b-6030-4813-a2db-5e65c785bf88.png',
    socials: {
      twitter: 'mjbotsrs',
      github: 'mjbots/moteus',
      mjbots: {
        label: 'Mjbots',
        icon: 'simple-icons:nuxtdotjs',
        href: 'https://mjbots.com/'
      }
    },
    github: {
      dir: './docs/content',
      branch: 'docus-docs',
      repo: 'moteus',
      owner: 'mjbots',
      edit: true
    },
    aside: {
      level: 0,
      collapsed: false,
      exclude: []
    },
    main: {
      padded: true,
      fluid: true
    },
    header: {
      logo: true,
      showLinkIcon: true,
      exclude: [],
      fluid: true
    }
  }
})
