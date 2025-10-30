# cmeng3952.github.io

这是我的个人博客，使用 Hugo 和 hugo-book 主题构建。

## 本地开发

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

## 技术栈

- [Hugo](https://gohugo.io/) - 静态网站生成器
- [Hugo Book Theme](https://github.com/alex-shpak/hugo-book) - 主题
- [GitHub Pages](https://pages.github.com/) - 托管服务
- [GitHub Actions](https://github.com/features/actions) - CI/CD

## 许可证

内容采用 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 许可。

