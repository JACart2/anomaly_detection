import os
import sys
import time
from pathlib import Path


def list_bag_dirs(base_dir: Path):
    if not base_dir.exists():
        return set()
    return {p.name for p in base_dir.iterdir() if p.is_dir()}


def main():
    bag_dir = Path(os.getenv("AAD_BAG_DIR", "/tmp/aad_api_bags"))
    wait_seconds = float(os.getenv("AAD_BAG_WAIT_SECONDS", "25"))
    poll_seconds = float(os.getenv("AAD_BAG_POLL_SECONDS", "1"))

    print(f"Watching bag directory: {bag_dir}")
    print(f"Timeout: {wait_seconds:.1f}s | Poll interval: {poll_seconds:.1f}s")

    before = list_bag_dirs(bag_dir)
    print(f"Existing bag directories before test: {sorted(before)}")
    print("Waiting for a new bag directory to appear...")

    deadline = time.time() + wait_seconds
    while time.time() < deadline:
        current = list_bag_dirs(bag_dir)
        new_dirs = sorted(current - before)
        if new_dirs:
            newest = new_dirs[-1]
            print(f"[PASS] New bag directory detected: {newest}")
            print(f"Full path: {bag_dir / newest}")
            print("Next command to inspect it:")
            print(f"ros2 bag info {bag_dir / newest}")
            sys.exit(0)
        time.sleep(poll_seconds)

    print("[FAIL] No new bag directory appeared before timeout.")
    sys.exit(1)


if __name__ == "__main__":
    main()
