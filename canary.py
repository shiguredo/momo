import argparse
import re
import subprocess
import sys

VERSION_FILE = "VERSION"

# canary リリースを実行できるブランチ
# Git Flow 運用のため develop で始まる作業ブランチは作らない前提とし、完全一致で判定する
CANARY_BRANCH = "develop"

# 基点バージョンの進め方
# keep はベースバージョンを据え置いたまま canary 番号だけを進める
BASE_UPDATE_KEEP = "keep"
# minor はマイナーを 1 増やしてパッチを 0 に戻す (--release)
BASE_UPDATE_MINOR = "minor"
# patch はパッチを 1 増やす (--hotfix)
BASE_UPDATE_PATCH = "patch"


def current_branch() -> str:
    # git 管理外や detached HEAD でも例外にせず、判定結果を文字列で受け取る
    # check=True にするとトレースバック付きで終了して usage 表示の経路に入らなくなる
    result = subprocess.run(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip()


def is_canary_version(version_content: str) -> bool:
    # 既に canary 番号が付いているかを判定する
    version_match = re.match(r"\d{4}\.\d+\.\d+-canary\.\d+", version_content.strip())
    return version_match is not None


def update_sdk_version(version_content: str, base_update: str) -> str:
    version_match = re.match(r"(\d{4})\.(\d+)\.(\d+)(-canary\.(\d+))?", version_content.strip())
    if version_match is None:
        raise ValueError("Invalid version format in VERSION file.")

    year = version_match.group(1)
    minor = int(version_match.group(2))
    patch = int(version_match.group(3))
    canary_suffix = version_match.group(4)
    canary_number = version_match.group(5)

    # 年は canary 実行者が VERSION を直接編集して変更するため、ここでは据え置く
    if base_update == BASE_UPDATE_KEEP:
        # ベースバージョンは据え置き、canary 番号だけを進める
        pass
    elif base_update == BASE_UPDATE_MINOR:
        minor = minor + 1
        patch = 0
    elif base_update == BASE_UPDATE_PATCH:
        patch = patch + 1
    else:
        # 未知の基点指定は基点を進めていないのに成功したように見えるため、
        # 無言で進めずに明示的に失敗させる
        raise ValueError(f"Unknown base update: base_update is '{base_update}'")

    major_minor_patch = f"{year}.{minor}.{patch}"

    if canary_suffix is None:
        new_version = f"{major_minor_patch}-canary.0"
    else:
        if canary_number is None:
            raise ValueError(
                f"Invalid canary suffix in VERSION file: content is '{version_content.strip()}'"
            )
        new_version = f"{major_minor_patch}-canary.{int(canary_number) + 1}"

    return new_version


def print_usage_and_error(parser: argparse.ArgumentParser, message: str, exit_code: int) -> None:
    # usage を出すためだけに argparse の parser.error は使わない
    # parser.error は必ず終了コード 2 で終了してしまい、経路ごとの終了コードを表現できない
    print(message, file=sys.stderr)
    parser.print_usage(sys.stderr)
    sys.exit(exit_code)


def write_version_file(filename: str, new_version: str, dry_run: bool) -> None:
    if dry_run:
        print(f"Dry run: The following changes would be written to {filename}:")
        print(new_version)
    else:
        with open(filename, "w") as file:
            file.write(new_version)
        print(f"{filename} updated.")


def git_operations(new_version: str, dry_run: bool) -> None:
    commit_message = f"[canary] Update VERSION to {new_version}"

    if dry_run:
        print(f"Dry run: Would execute git commit -am '{commit_message}'")
        print(f"Dry run: Would execute git tag {new_version}")
        print("Dry run: Would execute git push")
        print(f"Dry run: Would execute git push origin {new_version}")
    else:
        print(f"Executing: git commit -am '{commit_message}'")
        subprocess.run(["git", "commit", "-am", commit_message], check=True)

        print(f"Executing: git tag {new_version}")
        subprocess.run(["git", "tag", new_version], check=True)

        print("Executing: git push")
        subprocess.run(["git", "push"], check=True)

        print(f"Executing: git push origin {new_version}")
        subprocess.run(["git", "push", "origin", new_version], check=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Update VERSION file and push changes with git.",
        epilog=(
            "Without --release or --hotfix, canary number is incremented "
            "and the base version is kept as is."
        ),
    )
    base_group = parser.add_mutually_exclusive_group()
    base_group.add_argument(
        "--release",
        action="store_true",
        help="Start a canary release of the next minor version (patch is reset to 0).",
    )
    base_group.add_argument(
        "--hotfix",
        action="store_true",
        help="Start a canary release of the next patch version.",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Perform a dry run without making any changes."
    )
    args = parser.parse_args()

    # ブランチの検証は VERSION の状態やオプションの指定より優先し、VERSION の読み込み前に行う
    branch = current_branch()
    if branch != CANARY_BRANCH:
        print_usage_and_error(
            parser,
            f"Error: canary release must be run on the '{CANARY_BRANCH}' branch, "
            f"current branch is '{branch}'",
            1,
        )

    # Read the VERSION file
    with open(VERSION_FILE, "r") as file:
        version_content = file.read()

    base_update = BASE_UPDATE_KEEP
    if args.release:
        base_update = BASE_UPDATE_MINOR
    elif args.hotfix:
        base_update = BASE_UPDATE_PATCH

    # 既に canary 番号が付いた VERSION に基点の切り替えを指定すると、
    # 基点の二重進行になるため VERSION を書き換える前に止める
    if base_update != BASE_UPDATE_KEEP and is_canary_version(version_content):
        print_usage_and_error(
            parser,
            f"Error: --release and --hotfix cannot be used when VERSION already has "
            f"a canary number, current VERSION is '{version_content.strip()}'",
            1,
        )

    # 安定版 VERSION で基点の切り替えが指定されていない場合は何もせず usage を表示する
    if base_update == BASE_UPDATE_KEEP and not is_canary_version(version_content):
        print_usage_and_error(
            parser,
            f"Error: --release or --hotfix is required when VERSION has no canary number, "
            f"current VERSION is '{version_content.strip()}'",
            0,
        )

    # Read and update the VERSION file
    new_version = update_sdk_version(version_content, base_update)
    write_version_file(VERSION_FILE, new_version, args.dry_run)

    # Perform git operations
    git_operations(new_version, args.dry_run)


if __name__ == "__main__":
    main()
