"""맵/경로 데이터 디렉터리 결정.

환경변수 AUTORACE_DATA_DIR가 있으면 그 경로, 없으면 소스 트리의 autorace_main 디렉터리
(colcon --symlink-install 기준 설치본 파일이 소스로 연결되므로 맵핑 결과가 바로 소스에 저장된다).
"""
import os


def data_dir():
    env = os.environ.get('AUTORACE_DATA_DIR')
    if env:
        return os.path.expanduser(env)
    here = os.path.realpath(__file__)                 # .../src/autorace_main/autorace_main/paths.py
    return os.path.dirname(os.path.dirname(here))
