#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Git/GitHub 튜토리얼용 예제 파일
이 파일은 GRAPE 코드와는 전혀 관련이 없습니다.
"""

def hello_world():
    """간단한 인사 함수"""
    print("안녕하세요! Git/GitHub 튜토리얼입니다.")
    return "Hello, Git!"

def calculate_sum(a, b):
    """두 숫자를 더하는 함수"""
    return a + b

def print_git_commands():
    """Git 명령어들을 출력하는 함수"""
    commands = [
        "git add tutorial_example.py",
        "git commit -m '튜토리얼용 파일 추가'",
        "git push origin main",
        "git pull origin main",
        "git status"
    ]
    
    print("자주 사용하는 Git 명령어들:")
    for i, cmd in enumerate(commands, 1):
        print(f"{i}. {cmd}")

def main():
    """메인 함수"""
    print("=" * 50)
    hello_world()
    print("=" * 50)
    
    # 계산 예시
    result = calculate_sum(10, 20)
    print(f"10 + 20 = {result}")
    
    print_git_commands()
    
    print("\n이 파일을 수정해서 Git 연습을 해보세요!")
    print("마지막 수정: 2024년 12월")
    print("작성자: 튜토리얼 참가자")

if __name__ == "__main__":
    main() 