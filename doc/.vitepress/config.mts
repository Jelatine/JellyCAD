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
          { text: '使用脚本', link: '/guide/functions' },
          { text: '界面交互指南', link: '/guide/interaction' },
          { text: '圆角和倒角操作', link: '/guide/fillet_chamfer' },
          { text: '机器人开发指南', link: '/guide/robot_develop' },
        ]
      }
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/Jelatine/JellyCAD' }
    ]
  }
})
