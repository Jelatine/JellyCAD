import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  title: "JellyCAD",
  description: "An open-source programmable CAD software",
  base: '/JellyCAD/',
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    logo: '/favicon.ico',
    nav: [
      { text: '主页', link: '/' },
      { text: '文档', link: '/guide/install' }
    ],

    sidebar: [
      {
        text: '开始使用',
        items: [
          { text: '安装', link: '/guide/install' },
          { text: '内置函数', link: '/guide/functions' },
        ]
      }
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/Jelatine/JellyCAD' }
    ]
  }
})
