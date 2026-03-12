本项目包含三个主要部分：
- 📂 **Code/**: 存放所有的源代码。
- 📝 **Guide/**: 包含项目指导。
- 📚 **Materials/**: 相关的指导资料、论文和参考链接。

## 环境配置

### 1. 启动方法
开机时按F12，选择硬盘启动
### 2. 挂梯子操作
root@mobile:/home/fast125# source /etc/profile
root@mobile:/home/fast125# source ~/.bashrc
root@mobile:/home/fast125# crash
-----------------------------------------------
欢迎使用ShellCrash！		版本：1.9.4release
Mihomo服务没有运行（Redir模式），未设置开机启动！
TG频道：https://t.me/ShellClash
-----------------------------------------------
 1 启动/重启服务
 2 功能设置
 3 停止服务
 4 启动设置
 5 设置自动任务
 6 管理配置文件
 7 访问与控制
 8 工具与优化
 9 更新与支持
-----------------------------------------------
 0 退出脚本
请输入对应数字 > 1
-----------------------------------------------
服务已启动！                       
请使用 http://192.168.31.130:9999/ui 管理内置规则

### 3. 3.12进度
安装了nvidia 驱动
安装了ros
安装了docker
安装了 nvidia-container-toolkit
    - 验证：在 docker 中调用 nvidia-smi
        sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
        正常输出应显示你的 GPU 信息

按照rmua2026_setup_guide.html文档操作，step7的方式A可以正常启动了。上面的构建 basic_dev Docker 镜像（可选，用于提交测试）没有试过。