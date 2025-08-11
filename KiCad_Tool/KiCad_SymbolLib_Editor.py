import re
from pathlib import Path
import tkinter as tk
from tkinter import filedialog

def get_symbols_from_data(data):
    """データからメインシンボル一覧を抽出する（バリエーションは除外）"""
    # すべてのシンボルを取得
    all_symbols = re.findall(r'\(symbol\s+"([^"]+)"', data)
    # バリエーション（_数字_数字 形式）を除外してメイン定義のみを抽出
    main_symbols = []
    for symbol in all_symbols:
        # _数字_数字 パターンをチェック（例: "C_0_1", "MAX485ED_1_1"）
        if not re.search(r'_\d+_\d+$', symbol):
            main_symbols.append(symbol)
    
    return {i + 1: name for i, name in enumerate(main_symbols)}

def display_symbols(symbol_dict):
    """シンボル一覧を表示する"""
    print("\n現在のシンボル一覧")
    if not symbol_dict:
        print("  シンボルがありません")
        return
    for num, name in symbol_dict.items():
        print(f"{num}: {name}")

def delete_symbols_from_data(data, symbol_names):
    """データから指定されたシンボルとそのバリエーションを削除する"""
    updated_data = data
    for name in symbol_names:
        # メインシンボルとすべてのバリエーション（name_数字_数字）を削除
        patterns = [
            rf'\(symbol\s+"{re.escape(name)}"',  # メイン定義
            rf'\(symbol\s+"{re.escape(name)}_\d+_\d+"'  # バリエーション
        ]
        
        for pattern in patterns:
            # 括弧の開閉をカウントして完全なシンボル定義を見つける
            lines = updated_data.split('\n')
            new_lines = []
            skip_mode = False
            bracket_count = 0
            
            for line in lines:
                if not skip_mode:
                    # シンボル定義の開始を検出
                    if re.search(pattern, line):
                        skip_mode = True
                        bracket_count = line.count('(') - line.count(')')
                        # デバッグ出力
                        match = re.search(rf'"{re.escape(name)}(?:_\d+_\d+)?"', line)
                        if match:
                            found_name = match.group(0).strip('"')
                            print(f"🔍 削除開始: {found_name} (初期bracket_count: {bracket_count})")
                        if bracket_count <= 0:
                            skip_mode = False
                            print(f"単行で完結: {found_name}")
                        continue
                    else:
                        new_lines.append(line)
                else:
                    # スキップモード中：括弧のバランスを監視
                    bracket_count += line.count('(') - line.count(')')
                    if bracket_count <= 0:
                        skip_mode = False
                        print(f"削除完了: {found_name}")
                    # この行はスキップ
                    continue
            
            updated_data = '\n'.join(new_lines)
    
    return updated_data

def rename_symbol_in_data(data, old_name, new_name):
    """データ内のシンボル名とそのバリエーションを変更する"""
    updated_data = data
    
    # メイン定義をリネーム
    updated_data = re.sub(
        rf'\(symbol\s+"{re.escape(old_name)}"',
        f'(symbol "{new_name}"',
        updated_data
    )
    
    # バリエーション定義もリネーム（old_name_数字_数字 → new_name_数字_数字）
    updated_data = re.sub(
        rf'\(symbol\s+"{re.escape(old_name)}(_\d+_\d+)"',
        rf'(symbol "{new_name}\1"',
        updated_data
    )
    
    return updated_data

# Tkinterダイアログの非表示ウィンドウを作成
root = tk.Tk()
root.withdraw()

# ファイル選択ダイアログ
print("KiCadシンボルファイル（.kicad_sym）を選択してください...")
file_path_str = filedialog.askopenfilename(filetypes=[("KiCad Symbol Files", "*.kicad_sym")])

if not file_path_str:
    print("ファイルが選択されませんでした。処理を中止します。")
    exit()

file_path = Path(file_path_str)
with file_path.open("r", encoding="utf-8") as f:
    current_data = f.read()

print(f"\nファイル: {file_path.name}")

# メインループ
while True:
    # シンボル一覧の抽出と表示
    symbol_dict = get_symbols_from_data(current_data)
    display_symbols(symbol_dict)
    
    if not symbol_dict:
        print("すべてのシンボルが削除されました。")
        break
    
    # ユーザー入力（番号選択）
    print("\n操作したいシンボルの番号をカンマ区切りで入力してください")
    print("   終了するには '0' または 'q' を入力してください: ", end="")
    selected = input()
    
    # 終了チェック
    if selected.strip().lower() in ['0', 'q', 'quit', 'exit']:
        print("プログラムを終了します。")
        break
    
    # 選択された番号の処理
    try:
        selected_nums = [int(s.strip()) for s in selected.split(",") if s.strip().isdigit()]
        if not selected_nums:
            print("有効な番号が入力されませんでした。")
            continue
            
        selected_names = []
        for num in selected_nums:
            if num in symbol_dict:
                selected_names.append(symbol_dict[num])
            else:
                print(f"番号 {num} は存在しません。")
        
        if not selected_names:
            print("削除対象のシンボルがありません。")
            continue
            
    except ValueError:
        print("無効な入力です。数字をカンマ区切りで入力してください。")
        continue
    
    # 操作対象の表示と操作選択
    print(f"\n選択されたシンボル：")
    for name in selected_names:
        print(f" - {name}")
    
    print("\n操作を選択してください：")
    print("  d: 削除する")
    print("  r: リネームする") 
    print("  n: キャンセル")
    operation = input("選択 (d/r/n): ").strip().lower()
    
    if operation == 'd':
        # 削除処理
        current_data = delete_symbols_from_data(current_data, selected_names)
        # 即座にファイル保存
        with file_path.open("w", encoding="utf-8") as f:
            f.write(current_data)
        print(f"{len(selected_names)}個のシンボルを削除しました。")
    elif operation == 'r':
        # リネーム処理
        if len(selected_names) == 1:
            old_name = selected_names[0]
            print(f"\n'{old_name}' の新しい名前を入力してください")
            new_name = input("新しい名前: ").strip()
            
            if new_name and new_name != old_name:
                # 既存の名前との重複チェック
                existing_symbols = get_symbols_from_data(current_data)
                existing_names = [name for name in existing_symbols.values()]
                
                if new_name in existing_names:
                    print(f"\n'{new_name}' は既に存在します。")
                else:
                    current_data = rename_symbol_in_data(current_data, old_name, new_name)
                    # 即座にファイル保存
                    with file_path.open("w", encoding="utf-8") as f:
                        f.write(current_data)
                    print(f"'{old_name}' を '{new_name}' にリネームしました。")
            elif new_name == old_name:
                print("\n同じ名前です。変更されませんでした。")
            else:
                print("\n無効な名前です。")
        else:
            print("\nリネームは一度に1つのシンボルのみ可能です。")
    else:
        print("\n操作をキャンセルしました。")

# 最終的な保存
if current_data != open(file_path, "r", encoding="utf-8").read():
    with file_path.open("w", encoding="utf-8") as f:
        f.write(current_data)
    print(f"\n変更内容を {file_path.name} に保存しました。")
else:
    print("\n変更はありませんでした。")
