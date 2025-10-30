---
title: GitHub 命令行使用手册（Ubuntu/Linux）
weight: 4
bookToc: true
---

# GitHub 命令行使用手册（Ubuntu/Linux）

本手册详细介绍在 Ubuntu/Linux 系统下使用 Git 和 GitHub 的完整命令行操作指南。

## 1. 安装与配置

### 1.1 安装 Git

在 Ubuntu 系统上安装 Git：

```bash
# 更新包管理器
sudo apt update

# 安装 Git
sudo apt install git -y

# 验证安装
git --version
```

### 1.2 配置 Git 用户信息

```bash
# 配置用户名
git config --global user.name "你的用户名"

# 配置邮箱
git config --global user.email "your.email@example.com"

# 查看配置
git config --list

# 查看特定配置
git config user.name
git config user.email
```

### 1.3 配置默认编辑器

```bash
# 设置 vim 为默认编辑器
git config --global core.editor vim

# 或设置 nano
git config --global core.editor nano

# 或设置 VS Code
git config --global core.editor "code --wait"
```

### 1.4 配置默认分支名称

```bash
# 设置默认分支为 main
git config --global init.defaultBranch main
```

## 2. SSH 密钥配置

使用 SSH 连接 GitHub 更加安全便捷。

### 2.1 生成 SSH 密钥

```bash
# 生成新的 SSH 密钥（推荐使用 ed25519 算法）
ssh-keygen -t ed25519 -C "your.email@example.com"

# 如果系统不支持 ed25519，使用 RSA
ssh-keygen -t rsa -b 4096 -C "your.email@example.com"

# 按提示操作：
# 1. 按 Enter 使用默认路径 (~/.ssh/id_ed25519)
# 2. 输入密码（可选，建议设置）
# 3. 再次输入密码确认
```

### 2.2 启动 SSH 代理

```bash
# 启动 ssh-agent
eval "$(ssh-agent -s)"

# 添加 SSH 私钥到代理
ssh-add ~/.ssh/id_ed25519
```

### 2.3 添加 SSH 公钥到 GitHub

```bash
# 查看公钥内容
cat ~/.ssh/id_ed25519.pub

# 或直接复制到剪贴板（需要安装 xclip）
sudo apt install xclip
xclip -sel clip < ~/.ssh/id_ed25519.pub
```

然后在 GitHub 上添加 SSH 密钥：

1. 访问 https://github.com/settings/keys
2. 点击 "New SSH key"
3. 粘贴公钥内容
4. 点击 "Add SSH key"

### 2.4 测试 SSH 连接

```bash
# 测试 SSH 连接
ssh -T git@github.com

# 成功会显示：
# Hi username! You've successfully authenticated...
```

## 3. 基础 Git 操作

### 3.1 初始化仓库

```bash
# 创建新目录并初始化
mkdir my-project
cd my-project
git init

# 初始化时指定默认分支
git init -b main
```

### 3.2 克隆仓库

```bash
# 使用 HTTPS 克隆
git clone https://github.com/username/repository.git

# 使用 SSH 克隆（推荐）
git clone git@github.com:username/repository.git

# 克隆到指定目录
git clone git@github.com:username/repository.git my-folder

# 克隆指定分支
git clone -b branch-name git@github.com:username/repository.git

# 浅克隆（只克隆最近的提交历史）
git clone --depth 1 git@github.com:username/repository.git
```

### 3.3 查看状态

```bash
# 查看工作区状态
git status

# 简洁模式
git status -s

# 查看分支状态
git status -sb
```

### 3.4 添加文件

```bash
# 添加单个文件
git add filename.txt

# 添加多个文件
git add file1.txt file2.txt

# 添加所有文件
git add .

# 添加所有 .py 文件
git add *.py

# 交互式添加
git add -i

# 添加已修改和已删除的文件（不包括新文件）
git add -u
```

### 3.5 提交更改

```bash
# 提交更改
git commit -m "提交信息"

# 添加并提交（仅对已跟踪的文件）
git commit -am "提交信息"

# 修改最后一次提交
git commit --amend -m "新的提交信息"

# 空提交（用于触发 CI/CD）
git commit --allow-empty -m "Empty commit"
```

### 3.6 查看历史

```bash
# 查看提交历史
git log

# 单行显示
git log --oneline

# 显示最近 5 条
git log -5

# 图形化显示分支
git log --oneline --graph --all

# 查看文件的修改历史
git log -p filename

# 查看谁修改了文件的每一行
git blame filename

# 搜索提交信息
git log --grep="关键词"

# 查看某个作者的提交
git log --author="作者名"
```

## 4. 分支管理

### 4.1 创建和切换分支

```bash
# 查看所有分支
git branch

# 查看所有分支（包括远程）
git branch -a

# 创建新分支
git branch new-branch

# 切换到分支
git checkout new-branch

# 创建并切换到新分支
git checkout -b new-branch

# 使用新的 switch 命令切换分支
git switch new-branch

# 创建并切换（使用 switch）
git switch -c new-branch

# 从远程分支创建本地分支
git checkout -b local-branch origin/remote-branch
```

### 4.2 合并分支

```bash
# 合并指定分支到当前分支
git merge branch-name

# 不使用快进合并
git merge --no-ff branch-name

# 合并时创建提交信息
git merge branch-name -m "合并信息"

# 取消合并
git merge --abort
```

### 4.3 删除分支

```bash
# 删除本地分支
git branch -d branch-name

# 强制删除本地分支
git branch -D branch-name

# 删除远程分支
git push origin --delete branch-name

# 删除远程分支（旧语法）
git push origin :branch-name
```

### 4.4 重命名分支

```bash
# 重命名当前分支
git branch -m new-name

# 重命名其他分支
git branch -m old-name new-name

# 重命名远程分支（删除旧的，推送新的）
git push origin :old-name new-name
git push origin -u new-name
```

## 5. 远程仓库操作

### 5.1 管理远程仓库

```bash
# 查看远程仓库
git remote

# 查看远程仓库详细信息
git remote -v

# 添加远程仓库
git remote add origin git@github.com:username/repository.git

# 修改远程仓库 URL
git remote set-url origin git@github.com:username/repository.git

# 重命名远程仓库
git remote rename old-name new-name

# 删除远程仓库
git remote remove origin

# 查看远程仓库详细信息
git remote show origin
```

### 5.2 拉取和推送

```bash
# 拉取远程更改（不合并）
git fetch origin

# 拉取所有远程分支
git fetch --all

# 拉取并合并
git pull origin main

# 拉取并变基
git pull --rebase origin main

# 推送到远程仓库
git push origin main

# 首次推送并设置上游分支
git push -u origin main

# 推送所有分支
git push --all origin

# 推送标签
git push --tags

# 强制推送（谨慎使用）
git push -f origin main

# 删除远程分支
git push origin --delete branch-name
```

## 6. 撤销和恢复

### 6.1 撤销工作区更改

```bash
# 撤销单个文件的修改
git checkout -- filename

# 使用 restore 命令（推荐）
git restore filename

# 撤销所有修改
git checkout -- .
# 或
git restore .
```

### 6.2 撤销暂存区

```bash
# 取消暂存单个文件
git reset HEAD filename

# 使用 restore 命令（推荐）
git restore --staged filename

# 取消所有暂存
git reset HEAD
# 或
git restore --staged .
```

### 6.3 撤销提交

```bash
# 撤销最后一次提交，保留更改
git reset --soft HEAD~1

# 撤销最后一次提交，取消暂存
git reset HEAD~1

# 撤销最后一次提交，丢弃更改
git reset --hard HEAD~1

# 撤销到指定提交
git reset --hard commit-hash

# 创建反向提交
git revert HEAD

# 撤销指定提交
git revert commit-hash
```

### 6.4 恢复删除的文件

```bash
# 恢复已删除但未提交的文件
git checkout HEAD filename

# 从指定提交恢复文件
git checkout commit-hash filename

# 查看删除文件的提交
git log --diff-filter=D --summary
```

## 7. 标签管理

### 7.1 创建标签

```bash
# 创建轻量标签
git tag v1.0.0

# 创建附注标签（推荐）
git tag -a v1.0.0 -m "版本 1.0.0"

# 为指定提交创建标签
git tag -a v1.0.0 commit-hash -m "标签信息"
```

### 7.2 查看和管理标签

```bash
# 列出所有标签
git tag

# 列出匹配的标签
git tag -l "v1.*"

# 查看标签信息
git show v1.0.0

# 删除本地标签
git tag -d v1.0.0

# 删除远程标签
git push origin --delete v1.0.0
# 或
git push origin :refs/tags/v1.0.0
```

### 7.3 推送标签

```bash
# 推送单个标签
git push origin v1.0.0

# 推送所有标签
git push origin --tags

# 推送时包含标签
git push --follow-tags
```

## 8. 储藏（Stash）

### 8.1 储藏更改

```bash
# 储藏当前更改
git stash

# 储藏时添加描述
git stash save "工作进行中"

# 储藏包括未跟踪的文件
git stash -u

# 储藏包括忽略的文件
git stash -a
```

### 8.2 查看和应用储藏

```bash
# 查看储藏列表
git stash list

# 查看储藏内容
git stash show

# 查看详细差异
git stash show -p

# 应用最新的储藏
git stash apply

# 应用指定的储藏
git stash apply stash@{2}

# 应用并删除储藏
git stash pop

# 删除储藏
git stash drop stash@{0}

# 清空所有储藏
git stash clear
```

## 9. 高级操作

### 9.1 变基（Rebase）

```bash
# 变基到主分支
git rebase main

# 交互式变基（整理提交）
git rebase -i HEAD~3

# 变基时跳过冲突
git rebase --skip

# 取消变基
git rebase --abort

# 继续变基
git rebase --continue
```

### 9.2 拣选（Cherry-pick）

```bash
# 将指定提交应用到当前分支
git cherry-pick commit-hash

# 拣选多个提交
git cherry-pick commit1 commit2

# 拣选范围
git cherry-pick commit1..commit2

# 取消拣选
git cherry-pick --abort
```

### 9.3 子模块（Submodule）

```bash
# 添加子模块
git submodule add git@github.com:user/repo.git path/to/submodule

# 克隆包含子模块的仓库
git clone --recursive git@github.com:user/repo.git

# 初始化子模块
git submodule init

# 更新子模块
git submodule update

# 更新并初始化子模块
git submodule update --init --recursive

# 拉取所有子模块的最新更改
git submodule update --remote

# 删除子模块
git submodule deinit path/to/submodule
git rm path/to/submodule
rm -rf .git/modules/path/to/submodule
```

### 9.4 清理仓库

```bash
# 删除未跟踪的文件（预览）
git clean -n

# 删除未跟踪的文件
git clean -f

# 删除未跟踪的文件和目录
git clean -fd

# 删除包括忽略的文件
git clean -fx

# 清理过时的引用
git remote prune origin

# 垃圾回收
git gc

# 优化仓库
git gc --aggressive --prune=now
```

## 10. GitHub 特定操作

### 10.1 Fork 和 Pull Request 工作流

```bash
# 1. Fork 仓库（在 GitHub 网页上操作）

# 2. 克隆你的 fork
git clone git@github.com:your-username/repository.git
cd repository

# 3. 添加上游仓库
git remote add upstream git@github.com:original-owner/repository.git

# 4. 获取上游更新
git fetch upstream

# 5. 合并上游更新到本地主分支
git checkout main
git merge upstream/main

# 6. 创建功能分支
git checkout -b feature-branch

# 7. 进行修改并提交
git add .
git commit -m "添加新功能"

# 8. 推送到你的 fork
git push origin feature-branch

# 9. 在 GitHub 上创建 Pull Request

# 10. 同步 fork 与上游（保持更新）
git fetch upstream
git checkout main
git merge upstream/main
git push origin main
```

### 10.2 GitHub CLI（gh）

安装 GitHub CLI：

```bash
# Ubuntu/Debian
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
sudo apt update
sudo apt install gh

# 登录
gh auth login

# 创建仓库
gh repo create my-repo --public

# 克隆仓库
gh repo clone username/repository

# 查看 issues
gh issue list

# 创建 issue
gh issue create

# 查看 PR
gh pr list

# 创建 PR
gh pr create

# 查看 PR 详情
gh pr view 123

# 合并 PR
gh pr merge 123
```

## 11. 配置文件

### 11.1 .gitignore 文件

常见的 `.gitignore` 模板：

```gitignore
# Python
__pycache__/
*.py[cod]
*.so
.Python
env/
venv/
*.egg-info/

# Node.js
node_modules/
npm-debug.log
yarn-error.log

# IDE
.vscode/
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db

# Logs
*.log

# 环境变量
.env
.env.local

# 构建输出
dist/
build/
public/
*.min.js
*.min.css
```

### 11.2 .gitattributes 文件

```gitattributes
# 自动检测文本文件并规范化
* text=auto

# 强制 LF 换行符
*.sh text eol=lf
*.py text eol=lf

# 强制 CRLF 换行符
*.bat text eol=crlf

# 二进制文件
*.png binary
*.jpg binary
*.gif binary
*.ico binary
```

## 12. 常用技巧

### 12.1 别名（Alias）

```bash
# 设置常用别名
git config --global alias.st status
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.ci commit
git config --global alias.unstage 'reset HEAD --'
git config --global alias.last 'log -1 HEAD'
git config --global alias.lg "log --oneline --graph --all"

# 使用别名
git st
git co main
git lg
```

### 12.2 查看差异

```bash
# 查看工作区与暂存区的差异
git diff

# 查看暂存区与最后一次提交的差异
git diff --staged
# 或
git diff --cached

# 查看两个提交之间的差异
git diff commit1 commit2

# 查看两个分支的差异
git diff branch1 branch2

# 只显示文件名
git diff --name-only

# 显示统计信息
git diff --stat
```

### 12.3 搜索

```bash
# 在工作区搜索文本
git grep "搜索内容"

# 显示行号
git grep -n "搜索内容"

# 显示匹配数量
git grep -c "搜索内容"

# 在所有提交中搜索
git log -S "搜索内容"

# 搜索提交信息
git log --grep="搜索内容"
```

### 12.4 压缩提交

```bash
# 交互式变基，压缩最近 3 个提交
git rebase -i HEAD~3

# 在编辑器中，将要压缩的提交从 'pick' 改为 'squash' 或 's'
# 保存并退出，然后编辑合并后的提交信息
```

## 13. 故障排除

### 13.1 解决冲突

```bash
# 查看冲突文件
git status

# 手动编辑冲突文件，解决冲突标记：
# <<<<<<< HEAD
# 当前分支的内容
# =======
# 合并分支的内容
# >>>>>>> branch-name

# 标记冲突已解决
git add conflicted-file

# 继续合并
git commit

# 或取消合并
git merge --abort
```

### 13.2 撤销错误操作

```bash
# 查看引用日志（记录所有操作）
git reflog

# 恢复到之前的状态
git reset --hard HEAD@{2}

# 恢复已删除的分支
git checkout -b recovered-branch commit-hash
```

### 13.3 修复提交错误

```bash
# 修改最后一次提交的信息
git commit --amend -m "新的提交信息"

# 修改最后一次提交，添加遗漏的文件
git add forgotten-file
git commit --amend --no-edit

# 修改作者信息
git commit --amend --author="新作者 <email@example.com>"
```

## 14. 性能优化

### 14.1 大文件处理

```bash
# 安装 Git LFS
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt install git-lfs

# 初始化 Git LFS
git lfs install

# 跟踪大文件
git lfs track "*.psd"
git lfs track "*.zip"

# 查看跟踪的文件
git lfs ls-files

# 提交 .gitattributes
git add .gitattributes
git commit -m "Track large files with Git LFS"
```

### 14.2 仓库维护

```bash
# 检查仓库完整性
git fsck

# 统计仓库信息
git count-objects -vH

# 查找大文件
git rev-list --objects --all | \
  git cat-file --batch-check='%(objecttype) %(objectname) %(objectsize) %(rest)' | \
  awk '/^blob/ {print substr($0,6)}' | \
  sort --numeric-sort --key=2 | \
  tail -10

# 压缩和优化
git gc --aggressive --prune=now
```

## 15. 安全最佳实践

### 15.1 保护敏感信息

```bash
# 从历史中删除敏感文件
git filter-branch --force --index-filter \
  "git rm --cached --ignore-unmatch sensitive-file" \
  --prune-empty --tag-name-filter cat -- --all

# 使用 BFG Repo-Cleaner（更快）
java -jar bfg.jar --delete-files sensitive-file repo.git
cd repo.git
git reflog expire --expire=now --all
git gc --prune=now --aggressive
```

### 15.2 签名提交

```bash
# 配置 GPG 密钥
git config --global user.signingkey YOUR_GPG_KEY_ID

# 签名提交
git commit -S -m "签名的提交"

# 默认签名所有提交
git config --global commit.gpgsign true

# 验证签名
git log --show-signature
```

## 总结

本手册涵盖了在 Ubuntu/Linux 系统下使用 Git 和 GitHub 的主要命令和操作。建议：

1. **从基础开始**：先掌握基本的 add、commit、push、pull 操作
2. **使用 SSH**：配置 SSH 密钥，避免频繁输入密码
3. **善用分支**：使用分支进行功能开发和实验
4. **定期提交**：养成小步提交的习惯
5. **编写清晰的提交信息**：便于后期查看和维护
6. **学习撤销操作**：了解如何安全地撤销错误
7. **使用 .gitignore**：避免提交不必要的文件
8. **定期同步**：保持本地和远程仓库同步

## 参考资源

- [Git 官方文档](https://git-scm.com/doc)
- [GitHub 文档](https://docs.github.com/)
- [Pro Git 书籍](https://git-scm.com/book/zh/v2)
- [GitHub CLI 文档](https://cli.github.com/manual/)
- [Git LFS 文档](https://git-lfs.github.com/)

---

*持续更新中，欢迎提出建议和补充。*
