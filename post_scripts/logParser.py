import os
import re
import csv
import paramiko
import subprocess

LOG_DIR = '/home/rospi/Desktop/test_ws/src/post/post_scripts/logs'
REMOTE_LOG_DIR = '/home/rospi/Desktop/test_ws/loop'

# Log type should be named log-{ID}-{PI}.txt
LOG_PATTERN = re.compile(r'log-(?P<ID>[^-]+)-(?P<PI>[^.]+)\.txt')
DATA_PATTERN = re.compile(r'(\w+)=([^;]+)')
metrics = ['TIMESTAMP', 'PINAME', 'MSGID', 'OWNER', 'PREVLOC', 'NEXTLOC', 'INSTRUCTION_SET']
ip_list = [
    '172.23.254.18',
    '172.23.254.20',
    '172.23.254.22',
    '172.23.254.23',
    '172.23.254.24'
]
def fetch_logs_from_hosts(ip_list, username, password, remote_log_dir, local_save_dir):
    def download_directory(sftp, remote_dir, local_dir):
        """Recursively download a directory and its contents"""
        os.makedirs(local_dir, exist_ok=True)
        
        try:
            items = sftp.listdir_attr(remote_dir)
            for item in items:
                remote_path = f"{remote_dir}/{item.filename}"
                local_path = os.path.join(local_dir, item.filename)
                
                if item.st_mode & 0o040000:  # Check if it's a directory
                    print(f"Creating directory: {local_path}")
                    download_directory(sftp, remote_path, local_path)
                else:
                    print(f"Downloading file: {item.filename}")
                    sftp.get(remote_path, local_path)
        except Exception as e:
            print(f"Error processing {remote_dir}: {e}")
    
    for ip in ip_list:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            print(f"Connecting to {ip}...")
            ssh.connect(ip, username=username, password=password, timeout=10)
            sftp = ssh.open_sftp()
            
            # Create host-specific directory
            host_local_dir = os.path.join(local_save_dir, ip.replace('.', '_'))
            
            try:
                print(f"Downloading directory tree from {remote_log_dir} on {ip}...")
                download_directory(sftp, remote_log_dir, host_local_dir)
                print(f"Successfully downloaded all files and directories from {ip}")
            except FileNotFoundError:
                print(f"Remote directory {remote_log_dir} not found on {ip}")
            except PermissionError:
                print(f"Permission denied accessing {remote_log_dir} on {ip}")
            
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

def parse_log_file(LOG_DIR):
    new_metrics = {}
    for root, dirs, files in os.walk(LOG_DIR):
        for filename in files:
            match = LOG_PATTERN.match(filename)
            if match:
                log_id = match.group('ID')
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
                
                # Write all data at once
                write_header = not os.path.isfile(csv_path) or os.path.getsize(csv_path) == 0
                with open(csv_path, 'a', newline='') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=new_metrics[log_id])
                    if write_header:
                        writer.writeheader()
                    writer.writerows(log_data)

def parse_graveyard_logs(graveyard_dir):
    graveyard_metrics = {}
    for root, dirs, files in os.walk(graveyard_dir):
        for filename in files:
            match = LOG_PATTERN.match(filename)
            if match:
                log_id = match.group('ID')
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
                
                # Write graveyard data (overwrite, don't append)
                with open(csv_path, 'w', newline='') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=graveyard_metrics[log_id])
                    writer.writeheader()
                    writer.writerows(log_data)

def main():
    fetch_logs_from_hosts(ip_list, 'rospi', 'rospi', REMOTE_LOG_DIR, LOG_DIR)
    parse_log_file(LOG_DIR)
    parse_graveyard_logs('/home/rospi/Desktop/test_ws/graveyard')
    subprocess.run(['python3', 'preprocessData.py'])

if __name__ == "__main__":
    main()
    print("Log parsing completed.")
