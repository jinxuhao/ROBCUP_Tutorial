FROM palroboticssl/tiago_tutorials:noetic

# 安装 rocker
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install rocker

# 设置环境变量
ENV USER=root

# 运行 rocker 命令
CMD ["rocker", "--home", "--user", "--x11", "--privileged", "palroboticssl/tiago_tutorials:noetic", "--devices", "/dev/dri/card0"]
