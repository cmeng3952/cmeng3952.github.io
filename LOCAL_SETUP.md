# 本地开发环境设置

## Hugo 版本要求

hugo-book 主题需要 **Hugo Extended 0.146+** 版本。

### 检查当前版本

```bash
hugo version
```

### 升级 Hugo（如果需要）

#### Ubuntu/Debian

下载并安装最新版本：

```bash
# 设置版本号
HUGO_VERSION=0.146.0

# 下载
wget https://github.com/gohugoio/hugo/releases/download/v${HUGO_VERSION}/hugo_extended_${HUGO_VERSION}_linux-amd64.deb

# 安装
sudo dpkg -i hugo_extended_${HUGO_VERSION}_linux-amd64.deb

# 验证
hugo version
```

#### macOS

使用 Homebrew：

```bash
brew install hugo
```

#### Windows

使用 Chocolatey：

```bash
choco install hugo-extended
```

或从 [Hugo Releases](https://github.com/gohugoio/hugo/releases) 下载。

## 初始化子模块

如果你克隆了仓库，需要初始化主题子模块：

```bash
git submodule update --init --recursive
```

## 本地开发

安装正确版本后：

```bash
# 启动开发服务器（包含草稿）
hugo server -D

# 或不包含草稿
hugo server
```

然后访问 http://localhost:1313

## 如果本地 Hugo 版本较旧

如果无法升级本地 Hugo：

1. **不影响部署**：GitHub Actions 使用的是正确版本
2. **替代方案**：
   - 使用 Docker 运行 Hugo
   - 直接在 GitHub 上编辑文件，让 Actions 构建
   - 使用在线 IDE（如 GitHub Codespaces）

### 使用 Docker（推荐）

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

## 注意事项

- 确保使用 **Extended** 版本（支持 SCSS 编译）
- 不要提交 `public/` 目录（已在 .gitignore 中）
- 每次拉取代码后运行 `git submodule update --recursive`

