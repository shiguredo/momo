import argparse
import re
import subprocess

VERSION_FILE = "VERSION"


def update_sdk_version(version_content):
    version_match = re.match(r"(\d{4}\.\d+\.\d+)(-canary\.(\d+))?", version_content.strip())
    if version_match:
        major_minor_patch = version_match.group(1)
        canary_suffix = version_match.group(2)
        if canary_suffix is None:
            new_version = f"{major_minor_patch}-canary.0"
        else:
            canary_number = int(version_match.group(3))
            new_version = f"{major_minor_patch}-canary.{canary_number + 1}"
    else:
        raise ValueError("Invalid version format in VERSION file.")

    return new_version


def write_version_file(filename, new_version, dry_run):
    if dry_run:
        print(f"Dry run: The following changes would be written to {filename}:")
        print(new_version)
    else:
        with open(filename, "w") as file:
            file.write(new_version)
        print(f"{filename} updated.")


def git_operations(new_version, dry_run):
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


def main():
    parser = argparse.ArgumentParser(description="Update VERSION file and push changes with git.")
    parser.add_argument(
        "--dry-run", action="store_true", help="Perform a dry run without making any changes."
    )
    args = parser.parse_args()

    # Read and update the VERSION file
    with open(VERSION_FILE, "r") as file:
        version_content = file.read()
    new_version = update_sdk_version(version_content)
    write_version_file(VERSION_FILE, new_version, args.dry_run)

    # Perform git operations
    git_operations(new_version, args.dry_run)


if __name__ == "__main__":
    main()
