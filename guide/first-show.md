# 第一场演出

Python 3.12、Node.js 24、pnpm 10；PyBullet 无界面仿真，浏览器查看舞台。

## 准备舞台

```bash
bash scripts/install_environment.sh
source .venv/bin/activate
export PYTHONPATH="$PWD/src:$PWD/vendor/gym-pybullet-drones"
python service/server.py --host 127.0.0.1 --port 8765 --num-drones 128
```
打开 **http://127.0.0.1:8765/**。先在舞台选择编队与颜色，再通过 AGH 输入演出需求。中文文字编队使用 `DRONE_SHOW_CJK_FONT` 指定本机中文字体。

## 接上编排工具

```bash
export AGH_RUNTIME_DIR=/opt/agnes-harness/packages/cli/dist/local
node "$AGH_RUNTIME_DIR/agnes.mjs" serve
```
在 AGH 插件管理中安装仓库中的 `agh-plugin/`，启用 `@drone-show/agh-plugin`。插件默认连接 `http://127.0.0.1:8765`；独立端口通过 `DRONE_SHOW_SERVICE_URL` 设置。

## 检查编队与插件

```bash
python -m unittest discover -s tests/service -v
node --test tests/agh/plugin.test.mjs tests/agh/progress.test.mjs
```

## 修改舞台源码

默认舞台包含可直接使用的构建结果。修改 `web/stage.mjs` 后重新构建：

```bash
(cd tools/stage && npm ci)
node tools/build-stage.mjs
```
