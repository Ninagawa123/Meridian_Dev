# Meridian Markdown Generetor v0.0.1
# Meridianのローカル関数を含むc++ファイルから, mkdocs用のmdファイルを生成します.
# 使い方
# 1. Meridian_md_generator.py のあるディレクトリに移動します.
# 2. python Meridian_md_generator.py
# 3. 変換したいファイルのパスを入力します.
# 4. 指定されたC++ファイルと同じディレクトリに「markdown」フォルダが作成され、
#    その中に各関数のマークダウンファイルが生成されます。

import re
import os


def extract_functions(cpp_content):
    # 関数定義を抽出する正規表現パターン
    pattern = r'(\/\/\/[^}]*?\n)*\s*([\w:]+(?:\s+|\s*\*\s*)[\w:]+\s*\([^)]*\))\s*(?:const)?\s*{[^{}]*(?:{[^{}]*}[^{}]*)*}'

    # 予約語のリスト
    reserved_words = ['if', 'for', 'while', 'switch', 'else']

    functions = []
    for match in re.finditer(pattern, cpp_content, re.DOTALL):
        # 関数のシグネチャ部分を取得
        signature = match.group(2)
        # 関数名を取得
        function_name = signature.split('(')[0].split()[-1]

        # 関数名が予約語でない場合のみ追加
        if function_name not in reserved_words:
            functions.append(match.group(0))

    return functions


def generate_md(function_code):
    signature_match = re.search(
        r'([\w:]+(?:\s+|\s*\*\s*)[\w:]+\s*\([^)]*\))', function_code)
    if not signature_match:
        return None, "関数のシグネチャが見つかりません。"

    signature = signature_match.group(1)

    description = ""
    param_descriptions = []
    return_description = ""

    for line in function_code.split('\n'):
        if '///' in line:
            if '@brief' in line:
                description = line.split('@brief')[-1].strip()
            elif '@param' in line:
                param_descriptions.append(line.split('@param')[-1].strip())
            elif '@return' in line:
                return_description = line.split('@return')[-1].strip()

    md_content = f"""----  
<h2><b>{signature}</b></h2>
----  
{description}  
"""

    for param in param_descriptions:
        md_content += f"引数 : {param}  \n"

    if return_description:
        md_content += f"戻り値 : {return_description}  \n"

    md_content += """  
<br>  
```  
"""

    md_content += function_code.strip() + "\n```  "

    return signature, md_content


def sanitize_filename(filename):
    invalid_chars = r'[<>:"/\\|?*]'
    return re.sub(invalid_chars, '_', filename)


def clean_function_name(function_name):
    pattern = r'^(bool|void|uint64_t|short|UnionEEPROM|int|float|double|char|unsigned|long|size_t)_'
    return re.sub(pattern, '', function_name)


def process_cpp_file(file_path):
    cpp_dir = os.path.dirname(os.path.abspath(file_path))
    output_dir = os.path.join(cpp_dir, "markdown")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    with open(file_path, 'r', encoding='utf-8') as file:
        cpp_content = file.read()

    functions = extract_functions(cpp_content)

    for function_code in functions:
        signature, md_content = generate_md(function_code)

        if signature and md_content:
            function_name = signature.split('(')[0].split()[-1]
            clean_name = clean_function_name(function_name)
            filename = sanitize_filename(clean_name + '.md')
            output_file = os.path.join(output_dir, filename)
            with open(output_file, 'w', encoding='utf-8') as md_file:
                md_file.write(md_content)
            print(f"Generated MD file: {filename}")


def normalize_path(path):
    path = path.strip()
    expanded_path = os.path.expanduser(path)
    abs_path = os.path.abspath(expanded_path)
    return abs_path


if __name__ == "__main__":
    cpp_file_path = input("C++ファイルのパスを入力してください: ")

    cpp_file_path = normalize_path(cpp_file_path)

    if not os.path.exists(cpp_file_path):
        print(f"指定されたファイルが見つかりません: {cpp_file_path}")
    else:
        process_cpp_file(cpp_file_path)
        print("処理が完了しました。")
