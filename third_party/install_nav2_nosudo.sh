#!/bin/bash
# sudo 없이 Nav2 / slam_toolbox를 third_party/root 에 설치(.deb 다운로드 후 압축 해제)
# sudo가 가능하면 대신: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
set -e
cd "$(dirname "$0")"
mkdir -p debs root && cd debs
PKGS="ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox"
apt-get install -s $PKGS 2>/dev/null | grep "^Inst" | awk '{print $2}' \
  | grep -vE "fonts|ghostscript|gsfonts|libgs9|bzip2-doc|libbz2-dev|-perl$" > dl.txt
apt-get download $(cat dl.txt)
for d in *.deb; do dpkg -x "$d" ../root; done
echo "완료: source $(cd .. && pwd)/setup_extra.bash"
