from pathlib import Path
import os


def can_init():
    shell_path = Path(__file__).resolve().parent.joinpath('can_init.sh')
    os.system(shell_path)

if __name__ == '__main__':
    can_init()