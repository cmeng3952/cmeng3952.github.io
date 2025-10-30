---
title: 使用 Hugo Book 主题搭建 GitHub Pages 博客完整教程
weight: 2
bookToc: true
---

# 使用 Hugo Book 主题搭建 GitHub Pages 博客

本教程将详细介绍如何从零开始，使用 Hugo 静态网站生成器和 hugo-book 主题搭建一个托管在 GitHub Pages 上的个人博客。

## 前置要求

- Git 已安装
- 有 GitHub 账号
- 基本的命令行操作知识

## 第一步：安装 Hugo Extended

hugo-book 主题需要 Hugo Extended 版本 0.146 或更高版本。

### Ubuntu/Debian 系统

```bash
# 设置版本号
HUGO_VERSION=0.146.0

# 下载 Hugo Extended 版本
wget https://github.com/gohugoio/hugo/releases/download/v${HUGO_VERSION}/hugo_extended_${HUGO_VERSION}_linux-amd64.deb

# 安装
sudo dpkg -i hugo_extended_${HUGO_VERSION}_linux-amd64.deb

# 验证安装
hugo version
```

### macOS 系统

```bash
# 使用 Homebrew 安装
brew install hugo
```

### Windows 系统

```bash
# 使用 Chocolatey 安装
choco install hugo-extended
```

或从 [Hugo Releases](https://github.com/gohugoio/hugo/releases) 下载对应的安装包。

> [!IMPORTANT]
> 必须安装 **Extended** 版本，普通版本不支持 SCSS 编译。

## 第二步：创建 Hugo 站点

```bash
# 进入你想要存放博客的目录
cd ~/blog-theme

# 创建新的 Hugo 站点（使用你的 GitHub 用户名）
hugo new site 你的用户名.github.io

# 进入站点目录
cd 你的用户名.github.io

# 初始化 Git 仓库
git init

# 将默认分支重命名为 main
git branch -m main
```

## 第三步：添加 hugo-book 主题

有两种方式添加主题，推荐使用 Git Submodule 方式。

### 方式一：Git Submodule（推荐）

```bash
# 添加主题作为 submodule
git submodule add https://github.com/alex-shpak/hugo-book themes/hugo-book

# 或使用你 fork 的版本
git submodule add https://github.com/你的用户名/hugo-book themes/hugo-book
```

### 方式二：直接克隆

```bash
git clone https://github.com/alex-shpak/hugo-book themes/hugo-book
```

## 第四步：配置站点

编辑根目录下的 `config.toml` 文件：

```toml
baseURL = 'https://你的用户名.github.io/'
title = '你的博客名称'
theme = 'hugo-book'
languageCode = 'zh-cn'

# 启用 Git 信息（用于显示最后修改时间）
enableGitInfo = true
disablePathToLower = true

# Markdown 渲染配置
[markup]
[markup.goldmark.renderer]
  # 允许不安全的 HTML（用于某些 shortcodes）
  unsafe = true

[markup.tableOfContents]
  startLevel = 1
  endLevel = 6

[markup.highlight]
  style = 'monokai'

# 主题参数配置
[params]
  # 主题颜色：light, dark 或 auto
  BookTheme = 'auto'

  # 显示目录
  BookToC = true

  # 设置 favicon
  BookFavicon = 'favicon.png'

  # 指定文档内容目录
  BookSection = 'docs'

  # GitHub 仓库地址
  BookRepo = 'https://github.com/你的用户名/你的用户名.github.io'

  # 启用搜索功能
  BookSearch = true

  # 禁用评论（可以后续启用）
  BookComments = false

  # 日期格式
  BookDateFormat = '2006年1月2日'

  # 编辑链接模板
  BookEditLink = '{{ .Site.Params.BookRepo }}/edit/main/{{ .Path }}'

  # 最后修改链接模板
  BookLastChangeLink = '{{ .Site.Params.BookRepo }}/commit/{{ .GitInfo.Hash }}'

# 菜单配置
[[menu.after]]
  name = 'GitHub'
  url = 'https://github.com/你的用户名'
  weight = 10
```

## 第五步：创建内容结构

### 创建目录

```bash
mkdir -p content/docs content/posts
```

### 创建首页

创建 `content/_index.md`：

```markdown
---
title: 欢迎来到我的博客
type: docs
---

# 欢迎来到我的博客

这是使用 Hugo Book 主题构建的个人博客。

## 关于本站

这个博客用于记录我的学习笔记、技术文章和个人思考。

## 快速导航

- [文档](/docs) - 技术文档和笔记
- [博客文章](/posts) - 博客文章列表
```

### 创建文档区

创建 `content/docs/_index.md`：

```markdown
---
title: 文档
weight: 1
bookFlatSection: false
---

# 文档和笔记

这里存放各类技术文档和学习笔记。
```

创建 `content/docs/getting-started.md`：

```markdown
---
title: 开始使用
weight: 1
---

# 开始使用

欢迎来到我的博客！

## 导航

使用左侧的菜单可以浏览所有文档和文章。
```

### 创建博客区

创建 `content/posts/_index.md`：

```markdown
---
title: 博客文章
bookFlatSection: true
---

# 博客文章

这里是我的博客文章列表，按时间倒序排列。
```

创建 `content/posts/first-post.md`：

```markdown
---
title: 我的第一篇博客
date: 2025-10-30
draft: false
---

# 我的第一篇博客

欢迎来到我的博客！这是第一篇文章。
```

## 第六步：创建 .gitignore

创建 `.gitignore` 文件：

```
# Hugo 默认输出目录
/public/
/resources/_gen/
/assets/jsconfig.json

# Hugo 缓存目录
hugo_cache/

# 操作系统文件
.DS_Store
Thumbs.db

# 编辑器文件
.vscode/
.idea/
*.swp
*.swo
*~

# Node.js
node_modules/
package-lock.json

# 临时文件
*.log
.hugo_build.lock
```

## 第七步：配置 GitHub Actions 自动部署

创建 `.github/workflows/hugo.yml` 文件：

```yaml
# GitHub Pages 部署工作流
name: Deploy Hugo site to Pages

on:
  # 在推送到默认分支时运行
  push:
    branches:
      - main

  # 允许手动触发
  workflow_dispatch:

# 设置 GITHUB_TOKEN 的权限，允许部署到 GitHub Pages
permissions:
  contents: read
  pages: write
  id-token: write

# 只允许一个并发部署
concurrency:
  group: "pages"
  cancel-in-progress: false

# 默认使用 bash
defaults:
  run:
    shell: bash

jobs:
  # 构建任务
  build:
    runs-on: ubuntu-latest
    env:
      HUGO_VERSION: 0.146.0
    steps:
      - name: 安装 Hugo CLI
        run: |
          wget -O ${{ runner.temp }}/hugo.deb https://github.com/gohugoio/hugo/releases/download/v${HUGO_VERSION}/hugo_extended_${HUGO_VERSION}_linux-amd64.deb \
          && sudo dpkg -i ${{ runner.temp }}/hugo.deb          
      
      - name: 安装 Dart Sass
        run: sudo snap install dart-sass
      
      - name: 检出代码
        uses: actions/checkout@v4
        with:
          submodules: recursive
          fetch-depth: 0
      
      - name: 设置 Pages
        id: pages
        uses: actions/configure-pages@v5
      
      - name: 安装 Node.js 依赖
        run: "[[ -f package-lock.json || -f npm-shrinkwrap.json ]] && npm ci || true"
      
      - name: 使用 Hugo 构建
        env:
          HUGO_CACHEDIR: ${{ runner.temp }}/hugo_cache
          HUGO_ENVIRONMENT: production
          TZ: Asia/Shanghai
        run: |
          hugo \
            --gc \
            --minify \
            --baseURL "${{ steps.pages.outputs.base_url }}/"          
      
      - name: 上传构建产物
        uses: actions/upload-pages-artifact@v3
        with:
          path: ./public

  # 部署任务
  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: 部署到 GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

## 第八步：本地预览测试

在推送到 GitHub 之前，先在本地测试：

```bash
# 启动开发服务器
hugo server -D

# 在浏览器访问 http://localhost:1313
```

如果一切正常，按 `Ctrl+C` 停止服务器。

### 构建生产版本测试

```bash
# 构建站点
hugo --minify

# 检查 public 目录是否生成
ls public/
```

## 第九步：提交到 Git

```bash
# 添加所有文件
git add .

# 提交
git commit -m "Initial commit: Hugo blog with hugo-book theme"
```

## 第十步：推送到 GitHub

### 在 GitHub 创建仓库

1. 访问 https://github.com/new
2. 仓库名称必须是：`你的用户名.github.io`（例如：`cmeng3952.github.io`）
3. **不要**勾选"Initialize this repository with a README"
4. 点击 "Create repository"

### 推送代码

```bash
# 添加远程仓库（使用 HTTPS）
git remote add origin https://github.com/你的用户名/你的用户名.github.io.git

# 推送到 GitHub
git push -u origin main
```

如果使用 SSH：

```bash
git remote add origin git@github.com:你的用户名/你的用户名.github.io.git
git push -u origin main
```

## 第十一步：配置 GitHub Pages

1. 进入仓库的 **Settings** → **Pages**
   
   URL: `https://github.com/你的用户名/你的用户名.github.io/settings/pages`

2. 在 **Build and deployment** 部分：
   - **Source**: 选择 **GitHub Actions**

3. 保存设置

## 第十二步：等待部署完成

1. 访问 Actions 页面查看构建进度：
   
   `https://github.com/你的用户名/你的用户名.github.io/actions`

2. 等待工作流完成（通常 1-2 分钟）

3. 部署成功后访问你的博客：
   
   `https://你的用户名.github.io`

> [!NOTE]
> 第一次部署可能需要等待几分钟让 DNS 生效。

## 日常使用

### 创建新内容

```bash
# 创建文档
hugo new docs/my-article.md

# 创建博客文章
hugo new posts/my-post.md
```

### 本地预览

```bash
# 启动开发服务器（包含草稿）
hugo server -D

# 不包含草稿
hugo server
```

### 发布更新

```bash
# 编辑内容后提交
git add .
git commit -m "Add new post: xxx"
git push

# GitHub Actions 会自动重新部署
```

## 高级配置

### 使用 Markdown Alerts

Hugo 0.146+ 支持新的 markdown alerts 语法：

```markdown
> [!NOTE]
> 这是一个提示信息

> [!TIP]
> 这是一个小技巧

> [!IMPORTANT]
> 这是重要信息

> [!WARNING]
> 这是一个警告

> [!CAUTION]
> 这需要特别注意
```

### 自定义样式

创建 `assets/_custom.scss` 文件来添加自定义样式：

```scss
// 自定义颜色
:root {
  --body-font-color: #333;
  --color-link: #0066cc;
}

// 自定义样式
.custom-class {
  // 你的样式
}
```

### 添加 favicon

将你的 `favicon.png` 文件放到 `static/` 目录下。

### 配置多语言

在 `config.toml` 中添加：

```toml
[languages]
[languages.zh]
  languageName = 'Chinese'
  contentDir = 'content.zh'
  weight = 1

[languages.en]
  languageName = 'English'
  contentDir = 'content.en'
  weight = 2
```

然后创建对应的内容目录。

## 常见问题

### Q1: 构建失败，提示 Hugo 版本不够

**解决方案**：确保本地和 GitHub Actions 都使用 Hugo 0.146+ 版本。

### Q2: 主题样式不显示

**解决方案**：确保使用的是 Hugo Extended 版本。

### Q3: 推送到 GitHub 需要登录

**解决方案**：

```bash
# 配置 Git 用户信息
git config --global user.name "你的用户名"
git config --global user.email "你的邮箱"

# 使用 Personal Access Token 或 SSH key
```

### Q4: GitHub Pages 无法访问

**解决方案**：

1. 确保仓库名称为 `用户名.github.io`
2. 检查 GitHub Pages 设置是否选择了 "GitHub Actions"
3. 等待几分钟让 DNS 生效

### Q5: 子模块未正确克隆

**解决方案**：

```bash
# 初始化并更新子模块
git submodule update --init --recursive
```

## 使用 Docker（可选）

如果不想在本地安装 Hugo，可以使用 Docker：

```bash
# 启动开发服务器
docker run --rm -it \
  -v $(pwd):/src \
  -p 1313:1313 \
  klakegg/hugo:0.146.0-ext \
  server -D --bind 0.0.0.0

# 构建站点
docker run --rm -it \
  -v $(pwd):/src \
  klakegg/hugo:0.146.0-ext \
  --minify
```

## 相关资源

- [Hugo 官方文档](https://gohugo.io/documentation/)
- [Hugo Book 主题文档](https://github.com/alex-shpak/hugo-book)
- [GitHub Pages 文档](https://docs.github.com/pages)
- [GitHub Actions 文档](https://docs.github.com/actions)
- [Markdown 语法指南](https://www.markdownguide.org/)

## 总结

通过以上步骤，你已经成功搭建了一个：

- ✅ 使用 Hugo 构建的静态博客
- ✅ 使用 hugo-book 主题的优雅界面
- ✅ 托管在 GitHub Pages 的免费网站
- ✅ 通过 GitHub Actions 自动部署的工作流

现在你可以专注于创作内容，每次推送代码后，博客会自动更新！

---

*本教程基于实际搭建经验整理，如有问题欢迎反馈。*

