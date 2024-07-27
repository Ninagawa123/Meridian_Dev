# Meridian Markdown Files to yaml v0.0.1  
# 指定したディレクトリに含まれる.mdを見つけ, mkdocs用のyaml形式に抜き出します.  
# 使い方  
# 1. Meridian_md_generator.py のあるディレクトリに移動します.  
# 2. python Meridian_md_generator.py [.mdファイルを含む調べたいパス] で実行します.  
# 3. .mdファイルと同じディレクトリに, md_files_pickup.yamlファイルが生成されます.  
# 4. mkdocs.ymlに内容をコピペします.  
  
import os
import yaml
import argparse


def normalize_path(path):
    """パスを正規化する"""
    normalized = os.path.abspath(os.path.expanduser(path.rstrip(os.path.sep)))
    if normalized.endswith('demo'):
        return os.path.dirname(normalized)
    return normalized


def find_md_files(target_path):
    """指定されたパスから.mdファイルを再帰的に探索する"""
    md_files = []
    for root, _, files in os.walk(target_path):
        for file in files:
            if file.endswith('.md'):
                relative_path = os.path.relpath(
                    os.path.join(root, file), target_path)
                md_files.append(relative_path)
    return md_files


def generate_yaml(md_files, output_file):
    """YAMLファイルを生成する"""
    data = {
        'Local Function': [
            {os.path.splitext(file)[0]: f'LocalFunction/{file}'} for file in md_files
        ]
    }

    # カスタムYAML表現を作成
    yaml_content = "- 'Local Function':\n"
    for item in data['Local Function']:
        for key, value in item.items():
            yaml_content += f"        - '{key}' : '{value}'\n"

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(yaml_content)


def main():
    parser = argparse.ArgumentParser(
        description='Generate YAML file from markdown files.')
    parser.add_argument('path', help='Path to a directory or a .md file')
    args = parser.parse_args()

    target_path = normalize_path(args.path)

    if not os.path.exists(target_path):
        print(f"指定されたパスが見つかりません: {target_path}")
        return

    md_files = find_md_files(target_path)

    if not md_files:
        print("指定されたパスに.mdファイルが見つかりませんでした。")
        return

    output_dir = target_path
    output_file = os.path.join(output_dir, 'md_files_pickup.yaml')

    generate_yaml(md_files, output_file)
    print(f"YAMLファイルが生成されました: {output_file}")
    print(f"見つかった.mdファイルの数: {len(md_files)}")


if __name__ == "__main__":
    main()
