#!/usr/bin/env python3
"""quickstart 运行日志过滤器。

标准输入逐行读取，原样转发到标准输出（保持控制台实时输出），
同时把 ERROR 级别的行追加写入指定的日志文件。

在 quickstart.sh 中的位置：它被串在管道中间，上游是各 launch 的合并输出，
下游是终端。这样既保留控制台上的实时输出，又只把真正出错的行留存到文件，
避免把整份冗长的仿真日志全部落盘（tee 做不到按级别筛选）。

故障安全设计：
- 忽略 SIGINT/SIGTERM，保证 Ctrl+C 关闭过程中其他提示仍能正常显示并被转发
- 任何内部异常都不会中断转发，也不会提前退出（避免上游因管道破裂被 SIGPIPE 终止）
"""
import os
import re
import signal
import sys
from datetime import datetime

# 判定“错误行”的正则：大小写不敏感，匹配独立的 error/err/fatal 单词。
# 用 \b 词边界是为了避免把 error_code、errno、fatalistic 之类的子串误判为错误
LEVEL_PATTERN = re.compile(r'(?i)\b(error|err|fatal)\b')

# 日志头部/尾部的可读时间格式
TIMESTAMP_FORMAT = '%Y-%m-%d %H:%M:%S'


def open_log(log_path, command):
    """打开（追加）日志文件并写入抬头信息。

    任何失败都返回 None：调用方据此降级为“只转发、不落盘”，
    而不是让整个过滤器退出。
    """
    try:
        # 目标目录可能还不存在（例如首次运行、resources/logs 被清理过）
        directory = os.path.dirname(log_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        # buffering=1 即行缓冲：进程被强杀时已输出的行也不会丢
        log = open(log_path, 'a', encoding='utf-8', buffering=1)
        # 记录时间与完整命令，便于事后把这份错误摘录和某次运行对应起来
        log.write('# quickstart 运行日志（仅 ERROR 级别）\n')
        log.write(f'# 开始时间: {datetime.now().strftime(TIMESTAMP_FORMAT)}\n')
        log.write(f'# 命令: {command}\n\n')
        return log
    except OSError:
        return None


def main():
    # 忽略终止信号：Ctrl+C 时本进程不能先退出，否则管道下游提前关闭，
    # 上游还在输出的关闭提示会被丢弃，用户看不到
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    # 用法：quickstart_log_filter [日志路径] [原始命令]
    # 未给日志路径时退化为纯转发（相当于 tee 到终端）
    log_path = sys.argv[1] if len(sys.argv) > 1 else None
    # 第二个参数用于写抬头；缺省时用自身完整命令行兜底
    command = sys.argv[2] if len(sys.argv) > 2 else ' '.join(sys.argv)
    log = open_log(log_path, command) if log_path else None

    # 直接操作字节流：省掉一层文本解码的开销，也避免上游是非 UTF-8 时抛异常
    stdin = sys.stdin.buffer
    stdout = sys.stdout.buffer
    try:
        for raw in stdin:
            # errors='replace' 保证个别非法字节不会中断整体转发
            line = raw.decode('utf-8', errors='replace').rstrip('\n')
            try:
                # 先转发再判断是否记录：控制台输出优先级高于落盘
                stdout.write((line + '\n').encode('utf-8', errors='replace'))
                stdout.flush()
            except (BrokenPipeError, OSError):
                # 下游（终端/管道）已关闭，无需继续，直接收尾退出
                break
            if log is not None and LEVEL_PATTERN.search(line):
                try:
                    log.write(line + '\n')
                except OSError:
                    # 写日志失败就放弃日志功能，但转发继续进行
                    log = None
    except Exception:
        # 兜底：任何意外都不向调用方抛出，避免影响上游/下游进程
        pass
    finally:
        if log is not None:
            try:
                # 记录结束时间，便于判断这次运行是正常结束还是中途被打断
                log.write(
                    f'\n# 结束时间: {datetime.now().strftime(TIMESTAMP_FORMAT)}\n')
                log.close()
            except OSError:
                pass


if __name__ == '__main__':
    main()