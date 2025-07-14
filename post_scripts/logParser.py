import os
import re
import csv
import paramiko

LOG_DIR = '../post_logs'
REMOTE_LOG_DIR = '~/Desktop/test_ws/src/post/post_logs'
# Log type should be named log-{ID}-{PI}.txt
LOG_PATTERN = re.compile(r'log-(?P<ID>[^-]+)-(?P<PI>[^.]+)\.txt')
DATA_PATTERN = re.compile(r'KeyValue\(key=\'([^\']+)\',\s*value=\'([^\']+)\'\)')
metrics = ['TIMESTAMP', 'PINAME', 'MSGID', 'OWNER', 'PREVLOC', 'NEXTLOC', 'INSTRUCTION_SET']
ip_list = [
    '172.23.254.18',
    '172,23.254.22',
    '172.23.254.23',
    '172.23.254.24'
    ]

def fetch_logs_from_hosts(ip_list, username, password, remote_log_dir, local_save_dir):
    for ip in ip_list:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(ip, username=username, password=password)
            sftp = ssh.open_sftp()
            try:
                files = sftp.listdir(remote_log_dir)
                for file in files:
                    remote_file = os.path.join(remote_log_dir, file)
                    local_file = os.path.join(local_save_dir, file)
                    sftp.get(remote_file, local_file)
            finally:
                sftp.close()
        except Exception as e:
            print(f"Failed to fetch logs from {ip}: {e}")
        finally:
            ssh.close()

def parse_log_file(LOG_DIR):
    new_metrics = {}
    for fileame in os.listdir(LOG_DIR):
        log_data = []
        match = LOG_PATTERN.match(fileame)
        if match:
            log_id = match.group('ID')
            pi = match.group('PI')
            file_path = os.path.join(LOG_DIR, fileame)
            if os.path.isfile(LOG_DIR + '/' + f'log-{log_id}.csv'):
                with open(file_path, 'r') as file:
                    for line in file:
                        if line.strip():
                            parts = line.strip().split(',', len(metrics))
                            match_metrics = DATA_PATTERN.findall(line) if line else []
                            parts.append(match_metric[1] for match_metric in match_metrics)
                            if len(parts) == len(new_metrics[log_id]):
                                log_entry = dict(zip(new_metrics[log_id], parts))
                                log_data.append(log_entry)
                            with open(LOG_DIR + '/' + f'log-{log_id}.csv', 'a', newline='') as csvfile:
                                writer = csv.DictWriter(csvfile, fieldnames=new_metrics[log_id])
                                if csvfile.tell() == 0:
                                    writer.writeheader()
                                writer.writerow(log_entry)
            else:
                os.makefile(LOG_DIR + '/' + f'log-{log_id}.csv', 'w')
                # Parse the first log line to extract dynamic DATA keys
                new_metrics[log_id] = metrics.copy()
                with open(file, 'r') as file:
                    first_line = next((l for l in file if l.strip()), None)
                    match_metrics = DATA_PATTERN.findall(first_line) if first_line else []
                    new_metrics[log_id].append(match_metric[0] for match_metric in match_metrics)
                with open(file_path, 'r') as file:
                    for line in file:
                        if line.strip():
                            parts = line.strip().split(',', len(metrics))
                            match_metrics = DATA_PATTERN.findall(line) if line else []
                            parts.append(match_metric[1] for match_metric in match_metrics)
                            if len(parts) == len(new_metrics[log_id]):
                                log_entry = dict(zip(new_metrics[log_id], parts))
                                log_data.append(log_entry)
                            with open(LOG_DIR + '/' + f'log-{log_id}.csv', 'a', newline='') as csvfile:
                                writer = csv.DictWriter(csvfile, fieldnames=new_metrics[log_id])
                                if csvfile.tell() == 0:
                                    writer.writeheader()
                                writer.writerow(log_entry)
def main():
    fetch_logs_from_hosts(ip_list, 'rospi', 'rospi', REMOTE_LOG_DIR, LOG_DIR)
    parse_log_file(LOG_DIR)

if __name__ == "__main__":
    main()
    print("Log parsing completed.")