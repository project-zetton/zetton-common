# 代码贡献指南

[English](CONTRIBUTING.md) | 中文

- [代码贡献指南](#代码贡献指南)
  - [Pull Request 工作流程](#pull-request-工作流程)
    - [1. 复制和克隆](#1-复制和克隆)
    - [2. 配置 pre-commit](#2-配置-pre-commit)
    - [3. 创建开发分支](#3-创建开发分支)
    - [4. 提交代码并通过单元测试](#4-提交代码并通过单元测试)
    - [5. 将代码推送到远程](#5-将代码推送到远程)
    - [6. 创建 pull request](#6-创建-pull-request)
    - [7. 解决冲突](#7-解决冲突)
  - [指南](#指南)
    - [单元测试](#单元测试)
    - [文档渲染](#文档渲染)
  - [代码风格](#代码风格)
    - [Python](#python)
    - [C++ 和 CUDA](#c-和-cuda)
  - [PR 规范](#pr-规范)

欢迎来到 zetton-common 社区，我们致力于构建先进的机器人和计算机视觉基础库，并且欢迎各种贡献，包括但不限于：

**修复错误**

您可以直接发布 pull request 以修复代码或文档中的错别字。

修复代码实现中的错误步骤如下：

1. 如果修改涉及重大更改，则应首先创建一个 issue 并描述错误信息以及如何触发该错误。其他开发人员将与您讨论并提出适当的解决方案。

2. 修复错误并添加相应单元测试后发布 pull request。

**新功能或增强**

1. 如果修改涉及重大更改，您应该创建一个 issue 与我们的开发人员讨论提出适当的设计。

2. 实现新功能或增强后，请发布 pull request 并添加相应的单元测试。

**文档**

您可以直接发布 pull request 以修复文档。如果要添加文档，则应首先创建一个 issue 以检查其是否合理。

## Pull Request 工作流程

如果您不熟悉 pull request，请不要担心，以下指南将告诉您如何逐步创建 pull request。如果您想深入了解 pull request 的开发模式，可以参考 [官方文档](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/about-pull-requests)。

### 1. 复制和克隆

如果您是第一次提交 pull request，应该通过单击 GitHub 页面右上角的 **Fork** 按钮来复刻 zetton-common 代码仓库，并且 fork 的代码仓库将出现在您的 GitHub 个人资料下。
然后，您可以将代码仓库克隆到本地：

```shell
git clone git@github.com:{username}/zetton-common.git
```

之后，您应该将官方代码仓库添加为上游代码库

```bash
git remote add upstream git@github.com:project-zetton/zetton-common
```

通过 `git remote -v` 检查远程仓库是否已成功添加。

```bash
origin git@github.com:{username}/zetton-common.git (fetch)
origin git@github.com:{username}/zetton-common.git (push)
upstream git@github.com:project-zetton/zetton-common (fetch)
upstream git@github.com:project-zetton/zetton-common (push)
```

> 当我们使用 "git clone" 命令时，默认会创建一个名为 `origin` 的远程指向所克隆的仓库。至于 `upstream` ，则是我们手动添加以指向原始的目标仓库。当然，如果你不喜欢名称 `upstream` ，也可以自己命名。
> 通常情况下，我们会把代码推送到 `origin` 。如果推送的代码与官方（`upstream`）中最新代码发生冲突，则应从上游拉取最新代码以解决冲突，然后再次推送到 `origin`。提交的 Pull Request 将自动更新。

### 2. 配置 pre-commit

您应该在本地开发环境中配置 [pre-commit](https://pre-commit.com/#intro) ，以确保代码风格与 Project Zetton 的一致。

**注意**: 下面的代码应该在 zetton-common 目录下执行。

```shell
pip install -U pre-commit
pre-commit install
```

检查 pre-commit 是否成功配置，并安装 `.pre-commit-config.yaml` 中定义的钩子。

```shell
pre-commit run --all-files
```

如果安装过程被中断，可以重复运行 `pre-commit run ...` 来继续安装。

如果代码不符合代码风格规范，pre-commit 将会提出警告并自动修复部分错误。

如果我们想要跳过 pre-commit 钩子提交我们的代码，我们可以使用 `--no-verify` 选项（**仅用于临时提交**）。

```shell
git commit -m "xxx" --no-verify
```

### 3. 创建开发分支

在配置了 pre-commit 后，我们应该基于主分支创建一个新的特性或修复 bug 的开发分支。建议的分支名称是 `username/pr_name` 或者 `prefix/pr_name`，其中 `prefix` 与 commit msg 中的前缀一致，例如 `fix`、`feat`、`refactor` 等。

```shell
git checkout -b yhc/refactor_contributing_doc
# 或者
git checkout -b refactor/contributing_doc
```

在后续的开发中，如果本地仓库的主分支落后于 `upstream` 的主分支，则需要拉取上游仓库进行同步，然后执行上述命令：

```shell
git pull upstream master
```

### 4. 提交代码并通过单元测试

- zetton-common 引入了 mypy 进行静态类型检查，以增加代码的健壮性。因此，我们需要在代码中添加类型提示，并通过 mypy 检查。如果您不熟悉类型提示，可以参考 [这个教程](https://docs.python.org/3/library/typing.html)。

- 提交的代码应该通过单元测试

   ```shell
   # 通过所有单元测试
   pytest tests
   # 通过 runner 的单元测试
   pytest tests/test_runner/test_runner.py
   ```

   如果由于缺少依赖而导致单元测试失败，则可以按照 [指南](#单元测试) 安装依赖项。

- 如果修改或添加文档，则应根据 [指南](#文档渲染) 检查渲染结果。

### 5. 将代码推送到远程

在通过单元测试和预提交检查后，我们可以将本地提交推送到远程。您可以通过添加 `-u` 选项将本地分支与远程分支关联。

```shell
git push -u origin {branch_name}
```

这样下次就可以直接使用 `git push` 命令推送代码，无需指定分支或远程代码仓库。

### 6. 创建 pull request

1. 在 GitHub 的 pull request 界面中创建一个 pull request

2. 根据指南修改 PR 描述，以便其他开发人员更好地理解您的更改

有关 pull request 描述的详细信息，请参见 [pull request guidelines](#pr-规范)。

**注意**

1. pull request 说明应包含更改原因、更改内容和影响，并与相关问题相关联（请参阅 [文档](https://docs.github.com/en/issues/tracking-your-work-with-issues/linking-a-pull-request-to-an-issue)）

2. 如果这是您的第一次贡献，请签署 CLA 协议

3. 检查 pull request 是否通过 CI 测试：zetton-common 将在不同平台上运行单元测试来验证发布的 Pull Request 是否正确（Linux、Window、Mac），基于不同版本的 Python、PyTorch 和 CUDA。我们可以点击上面图像中的“详情”来查看具体测试信息，以便我们进行修改代码。

4. 如果 Pull Request 通过了 CI 测试，则您可以等待其他开发人员进行审查。然后根据评审人员的评论修改代码，并重复步骤 [4](#4-提交代码并通过单元测试) 与步骤 [5](#5-将代码推送到远程)，直到所有评审者批准为止。然后，我们会尽快合并它。

### 7. 解决冲突

如果你本地分支与 `upstream` 最新主干分支存在冲突，则需要解决它们。有两种方法可做到这一点：

```shell
git fetch --all --prune
git rebase upstream/master
```

或者

```shell
git fetch --all --prune
git merge upstream/master
```

如果你很擅长处理冲突，则可以使用 rebase 来解决冲突，因为这样会保持你提交日志整洁清晰。如果你对 `rebase` 不熟悉，则可以使用 `merge` 来解决冲突。

## 指南

### 单元测试

如果由于缺少某些依赖项而无法运行某些模块的单元测试，则可以尝试安装所需的依赖项。例如，如果我们想要运行的单元测试依赖于`libturbojpeg` 和 `ffmpeg`，则需要安装：

```shell
# Linux
sudo apt-get update -y
sudo apt-get install -y libturbojpeg
sudo apt-get install -y ffmpeg
# Windows
conda install ffmpeg
```

我们还应确保提交的代码不会降低单元测试覆盖率，我们可以运行以下命令来检查单元测试覆盖率：

```shell
python -m coverage run -m pytest /path/to/test_file
python -m coverage html
# 在 htmlcov/index.html 中检查文件
```

### 文档渲染

如果修改/添加了文档，我们应该检查渲染结果。我们可以安装依赖项并运行以下命令来渲染文档并检查结果：

```shell
pip install -r requirements/docs.txt
cd docs/zh_cn/
# 或者 docs/en
make html
# 检查 ./docs/zh_cn/_build/html/index.html 文件中的内容。
```

## 代码风格

### Python

我们采用 [PEP8](https://www.python.org/dev/peps/pep-0008/) 作为首选代码风格。
我们使用以下工具进行静态分析和格式化：

- [flake8](https://github.com/PyCQA/flake8)：一些 linter 工具的包装器。

- [isort](https://github.com/timothycrosley/isort)：一个用于排序导入语句的 Python 实用程序。

- [yapf](https://github.com/google/yapf)：Python 文件格式化程序。

- [codespell](https://github.com/codespell-project/codespell)：修复文本文件中常见拼写错误的 Python 实用程序。

- [mdformat](https://github.com/executablebooks/mdformat)：Mdformat 是一个有主见的 Markdown 格式化程序，可用于强制执行 Markdown 文件中一致的样式规范。

- [docformatter](https://github.com/myint/docformatter)：格式化 docstring 的工具。

yapf 和 isort 的样式配置可以在 [setup.cfg](./setup.cfg) 中找到。

我们使用 [pre-commit hook](https://pre-commit.com/)，每次提交时都会自动检查和格式化 `flake8`、`yapf`、`isort`、`trailing whitespaces`、 `markdown files`, 自动修复 `end-of-files`, `double-quoted strings`, `python encoding pragma`, `mixed-line-ending`, 并自动对 `requirments.txt` 进行排序。

预提交钩子的配置存储在 [.pre-commit-config](./.pre-commit-config.yaml) 中。

### C++ 和 CUDA

我们遵循 [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)。

## PR 规范

1. 使用 [pre-commit](https://pre-commit.com) hook 以避免代码风格问题

2. 一个 PR 应该仅对应一个短期分支

3. 在单个 PR 中仅完成一项更改，请避免大型 PR

   - Bad：实现 Faster R-CNN
   - Acceptable：给 Faster R-CNN 添加一个 box head
   - Good：给 box head 增加一个参数来支持自定义的 conv 层数

4. 提供清晰明了且有意义的提交消息，格式请参照 [AngularJS Commit Mesage Format](https://github.com/angular/angular/blob/main/CONTRIBUTING.md#-commit-message-format)

5. 提供清晰而有意义的 PR 描述

   - 任务名称应在标题中说明。通常格式为：`[Prefix] PR 的简短描述 (Suffix)`

   - Prefix:

     - 新增功能 `[Feature]`

     - 修复 bug `[Fix]`

     - 相关文档 `[Docs]`

     - 正在开发 `[WIP]` （暂不审核）

   - 在简要说明中介绍主要变更、结果及其对其他模块产生影响

   - 将相关问题和 pull request 与里程碑关联起来
