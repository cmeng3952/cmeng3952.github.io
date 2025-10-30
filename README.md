# cmeng3952.github.io

这是我的个人博客，使用 Hugo 和 hugo-book 主题构建。

🌐 **在线访问**: https://cmeng3952.github.io

## 本地开发

### 前置要求

- Hugo Extended 0.146.0+
- Git

### 启动本地服务器

```bash
hugo server -D
```

然后在浏览器访问 http://localhost:1313

### 创建新文章

创建文档：
```bash
hugo new docs/your-article.md
```

创建博客文章：
```bash
hugo new posts/your-post.md
```

### 构建站点

```bash
hugo --minify
```

构建的静态文件会输出到 `public/` 目录。

## 部署

本博客使用 GitHub Actions 自动部署到 GitHub Pages。

每次推送到 `main` 分支时，会自动触发构建和部署。

## 内容结构

```
content/
├── _index.md           # 首页
├── docs/               # 文档区
│   ├── _index.md
│   ├── hugo-blog-setup.md    # Hugo 博客搭建完整教程
│   └── getting-started.md    # 使用指南
└── posts/              # 博客区
    ├── _index.md
    └── first-post.md
```

## 技术栈

- [Hugo](https://gohugo.io/) v0.146.0+ - 静态网站生成器
- [Hugo Book Theme](https://github.com/alex-shpak/hugo-book) - 主题
- [GitHub Pages](https://pages.github.com/) - 托管服务
- [GitHub Actions](https://github.com/features/actions) - CI/CD

## 主要特性

- ✨ 简洁优雅的书籍式主题
- 🔍 全站搜索功能
- 🌙 深色/浅色模式切换
- 📱 移动端自适应
- 📖 目录导航
- 🚀 自动化部署
- ⚡ 快速构建和加载

## 详细文档

查看博客中的 [Hugo 博客搭建教程](https://cmeng3952.github.io/docs/hugo-blog-setup/) 了解完整的搭建流程。

## 许可证

- 网站内容采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 许可
- 代码采用 MIT 许可

