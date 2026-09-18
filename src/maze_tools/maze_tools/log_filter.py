#!/usr/bin/env python3
"""quickstart 运行日志过滤器。

标准输入逐行读取，原样转发到标准输出（保持控制台实时输出），
同时把 ERROR 级别的行追加写入指定的日志文件。

故障安全设计：
- 忽略 SIGINT/SIGTERM，保证 Ctrl+C 关闭过程中其他提示仍能正常显示并被转发
- 任何内部异常都不会中断转发，也不会提前退出（避免上游因管道破裂被 SIGPIPE 终止）
"""
import os
import re
import signal
import sys
from datetime import datetime

LEVEL_PATTERN = re.compile(r'(?i)\b(error|err|fatal)\b')

TIMESTAMP_FORMAT = '%Y-%m-%d %H:%M:%S'


def open_log(log_path, command):
    try:
        directory = os.path.dirname(log_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        log = open(log_path, 'a', encoding='utf-8', buffering=1)
        log.write('# quickstart 运行日志（仅 ERROR 级别）\n')
        log.write(f'# 开始时间: {datetime.now().strftime(TIMESTAMP_FORMAT)}\n')
        log.write(f'# 命令: {command}\n\n')
        return log
    except OSError:
        return None


def main():
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    log_path = sys.argv[1] if len(sys.argv) > 1 else None
    command = sys.argv[2] if len(sys.argv) > 2 else ' '.join(sys.argv)
    log = open_log(log_path, command) if log_path else None

    stdin = sys.stdin.buffer
    stdout = sys.stdout.buffer
    try:
        for raw in stdin:
            line = raw.decode('utf-8', errors='replace').rstrip('\n')
            try:
                stdout.write((line + '\n').encode('utf-8', errors='replace'))
                stdout.flush()
            except (BrokenPipeError, OSError):
                break
            if log is not None and LEVEL_PATTERN.search(line):
                try:
                    log.write(line + '\n')
                except OSError:
                    log = None
    except Exception:
        pass
    finally:
        if log is not None:
            try:
                log.write(
                    f'\n# 结束时间: {datetime.now().strftime(TIMESTAMP_FORMAT)}\n')
                log.close()
            except OSError:
                pass


if __name__ == '__main__':
    main()