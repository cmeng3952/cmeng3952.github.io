# 部署指南

## 步骤 1：在 GitHub 上创建仓库

1. 访问 https://github.com/new
2. 仓库名称必须是：`cmeng3952.github.io`
3. **不要**初始化 README、.gitignore 或 license（我们已经创建好了）
4. 点击 "Create repository"

## 步骤 2：推送代码到 GitHub

在终端中执行以下命令（在博客目录中）：

```bash
# 添加远程仓库
git remote add origin https://github.com/cmeng3952/cmeng3952.github.io.git

# 推送到 GitHub
git push -u origin main
```

如果使用 SSH：
```bash
git remote add origin git@github.com:cmeng3952/cmeng3952.github.io.git
git push -u origin main
```

## 步骤 3：配置 GitHub Pages

1. 进入仓库设置：https://github.com/cmeng3952/cmeng3952.github.io/settings/pages

2. 在 "Build and deployment" 部分：
   - **Source**: 选择 "GitHub Actions"
   
3. 保存设置

## 步骤 4：触发部署

代码推送后，GitHub Actions 会自动开始构建和部署：

1. 查看 Actions 进度：https://github.com/cmeng3952/cmeng3952.github.io/actions

2. 等待部署完成（通常需要 1-2 分钟）

3. 访问你的博客：https://cmeng3952.github.io

## 后续更新

每次修改内容后：

```bash
git add .
git commit -m "你的提交信息"
git push
```

GitHub Actions 会自动重新部署。

## 本地预览

在推送前，可以本地预览：

```bash
# 启动本地服务器
hugo server -D

# 在浏览器访问 http://localhost:1313
```

## 常见问题

### 问题 1：推送时要求登录

使用以下命令配置 Git 凭据：
```bash
git config --global user.name "你的用户名"
git config --global user.email "你的邮箱"
```

对于 HTTPS，可能需要使用 Personal Access Token。

### 问题 2：Actions 构建失败

1. 检查 Actions 日志查看错误信息
2. 确保 GitHub Pages 设置为使用 "GitHub Actions"
3. 检查仓库权限设置

### 问题 3：网站无法访问

1. 确保仓库名称正确为 `cmeng3952.github.io`
2. 等待几分钟让 DNS 生效
3. 检查 GitHub Pages 设置是否正确

## 自定义域名（可选）

如果你有自己的域名：

1. 在仓库根目录创建 `static/CNAME` 文件，内容为你的域名
2. 在域名 DNS 设置中添加 CNAME 记录指向 `cmeng3952.github.io`
3. 在 GitHub Pages 设置中配置自定义域名

## 更多资源

- [Hugo 文档](https://gohugo.io/documentation/)
- [Hugo Book 主题文档](https://github.com/alex-shpak/hugo-book)
- [GitHub Pages 文档](https://docs.github.com/pages)
- [GitHub Actions 文档](https://docs.github.com/actions)

