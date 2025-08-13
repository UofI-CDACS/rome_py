import os
import re
import csv
import paramiko
import subprocess
import argparse
LOG_DIR = '/home/rospi/Desktop/test_ws/src/post/post_scripts/logs'

# Log type should be named log-{ID}-{PI}.txt

DATA_PATTERN = re.compile(r'(\w+)=([^;]+)')
metrics = ['TIMESTAMP', 'PINAME', 'MSGID', 'OWNER', 'PREVLOC', 'NEXTLOC', 'INSTRUCTION_SET', 'CPU_PERCENT', 'CPU_TEMP', 'RAM_PERCENT', 'BYTES_SENT_MB', 'BYTES_RECV_MB', 'PARCEL_SIZE_MB']
ip_list = [
    '172.23.254.24',
    '172.23.254.22',
    '172.23.254.23',
    '172.23.254.18'
]
pi_names = [
    'rospi-1-desktop',
    'rospi-2-desktop',
    'rospi-3-desktop',
    'rospi-4-desktop'
]

def fetch_logs_from_hosts(ip_list, username, password, remote_log_dir, local_save_dir,identifier=None):
    def download_file(sftp, remote_file_path, local_file_path):
        """Download a single file from remote host"""
        try:
            # Create local directory if it doesn't exist
            local_dir = os.path.dirname(local_file_path)
            os.makedirs(local_dir, exist_ok=True)
            
            print(f"Downloading file: {remote_file_path}")
            sftp.get(remote_file_path, local_file_path)
            print(f"Successfully downloaded: {os.path.basename(remote_file_path)}")
        except Exception as e:
            print(f"Error downloading {remote_file_path}: {e}")

    for i, ip in enumerate(ip_list):
        pi_log_dir = remote_log_dir + f'/rospi_{i+1}/log-{identifier}-{pi_names[i]}.txt'
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            print(f"Connecting to {ip}...")
            ssh.connect(ip, username=username, password=password, timeout=10)
            sftp = ssh.open_sftp()
            
            # Create local file path instead of directory
            local_file_path = os.path.join(local_save_dir, f'log-{identifier}-{pi_names[i]}.txt')
            
            try:
                print(f"Downloading file from {pi_log_dir} on {ip}...")
                download_file(sftp, pi_log_dir, local_file_path)

                print(f"Successfully downloaded file from {ip}")
            except FileNotFoundError:
                print(f"Remote file {pi_log_dir} not found on {ip}")
            except PermissionError:
                print(f"Permission denied accessing {pi_log_dir} on {ip}")
            sftp.close()
            print(f"Successfully processed {ip}")
        except paramiko.AuthenticationException:
            print(f"Authentication failed for {ip}")
        except paramiko.SSHException as e:
            print(f"SSH connection failed for {ip}: {e}")
        except Exception as e:
            print(f"Failed to fetch logs from {ip}: {e}")
        finally:
            ssh.close()

def parse_log_file(LOG_DIR,identifier=None):
    new_metrics = {}
    csv_files_written = set()  # Track which CSV files we've already written to
    full_pattern = r'log-' + identifier + r'-(?P<PI>[^.]+)\.txt'
    LOG_PATTERN = re.compile(full_pattern)
    for root, dirs, files in os.walk(LOG_DIR):
        for filename in files:
            match = LOG_PATTERN.match(filename)
            if match:
                log_id = identifier
                pi = match.group('PI')
                file_path = os.path.join(root, filename)
                csv_path = os.path.join(LOG_DIR, f'log-{log_id}.csv')
                
                # Initialize metrics for this log_id if not exists
                if log_id not in new_metrics:
                    new_metrics[log_id] = metrics.copy()
                    # Parse first line to extract dynamic DATA keys
                    with open(file_path, 'r') as file:
                        first_line = next((l for l in file if l.strip()), None)
                        if first_line:
                            match_metrics = DATA_PATTERN.findall(first_line)
                            new_metrics[log_id].extend([match_metric[0] for match_metric in match_metrics])
                
                log_data = []
                with open(file_path, 'r') as file:
                    for line in file:
                        if line.strip():
                            parts = line.strip().split(',')
                            parts = parts[:len(metrics)]
                            match_metrics = DATA_PATTERN.findall(line)
                            parts.extend(match_metric[1].strip() for match_metric in match_metrics)
                            if len(parts) == len(new_metrics[log_id]):
                                log_entry = dict(zip(new_metrics[log_id], parts))
                                log_data.append(log_entry)
                
                # Determine write mode: overwrite first time, append subsequent times
                write_mode = 'w' if csv_path not in csv_files_written else 'a'
                write_header = csv_path not in csv_files_written
                
                with open(csv_path, write_mode, newline='') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=new_metrics[log_id])
                    if write_header:
                        writer.writeheader()
                    writer.writerows(log_data)
                
                csv_files_written.add(csv_path)

def parse_graveyard_logs(graveyard_dir,identifier=None):
    graveyard_metrics = {}
    csv_files_written = set()  # Track which CSV files we've already written to
    full_pattern = r'log-' + identifier + r'-(?P<PI>[^.]+)\.txt'
    LOG_PATTERN = re.compile(full_pattern)
    for root, dirs, files in os.walk(graveyard_dir):
        for filename in files:
            match = LOG_PATTERN.match(filename)
            if match:
                log_id = identifier
                pi = match.group('PI')
                file_path = os.path.join(root, filename)
                csv_path = os.path.join(graveyard_dir, f'graveyard-{log_id}.csv')
                
                # Initialize metrics for this log_id if not exists
                if log_id not in graveyard_metrics:
                    graveyard_metrics[log_id] = metrics.copy()
                    # Parse first line to extract dynamic DATA keys
                    with open(file_path, 'r') as file:
                        first_line = next((l for l in file if l.strip()), None)
                        if first_line:
                            match_metrics = DATA_PATTERN.findall(first_line)
                            graveyard_metrics[log_id].extend([match_metric[0] for match_metric in match_metrics])
                
                log_data = []
                with open(file_path, 'r') as file:
                    for line in file:
                        if line.strip():
                            parts = line.strip().split(',')
                            parts = parts[:len(metrics)]
                            match_metrics = DATA_PATTERN.findall(line)
                            parts.extend(match_metric[1].strip() for match_metric in match_metrics)
                            if len(parts) == len(graveyard_metrics[log_id]):
                                log_entry = dict(zip(graveyard_metrics[log_id], parts))
                                log_data.append(log_entry)
                
                # Determine write mode: overwrite first time, append subsequent times
                write_mode = 'w' if csv_path not in csv_files_written else 'a'
                write_header = csv_path not in csv_files_written
                
                with open(csv_path, write_mode, newline='') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=graveyard_metrics[log_id])
                    if write_header:
                        writer.writeheader()
                    writer.writerows(log_data)
                
                csv_files_written.add(csv_path)

def main(owner=None, instruction_set=None):
    REMOTE_LOG_DIR = f'/home/rospi/Desktop/test_ws/{instruction_set}'
    fetch_logs_from_hosts(ip_list, 'rospi', 'rospi', REMOTE_LOG_DIR, LOG_DIR, identifier=owner)
    parse_log_file(LOG_DIR, identifier=owner)
    parse_graveyard_logs('/home/rospi/Desktop/test_ws/graveyard', identifier=owner)
    subprocess.run(['python3', '/home/rospi/Desktop/test_ws/src/post/post_scripts/preprocessData.py', '--owner', owner])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse logs from remote Raspberry Pis")
    parser.add_argument('--owner', type=str, help='Owner ID to filter logs')
    parser.add_argument('--instruction_set', type=str, help='Instruction set to filter logs')
    args = parser.parse_args()
    main(owner=args.owner, instruction_set=args.instruction_set)
    print("Log parsing completed.")
