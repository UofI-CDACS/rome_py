import pandas as pd
import os
import glob

# Get all log files and their identifiers
log_files = glob.glob("/home/rospi/Desktop/test_ws/src/post/post_scripts/logs/log-*.csv")
identifiers = [os.path.basename(f).replace("log-", "").replace(".csv", "") for f in log_files]
# Process each identifier
for identifier in identifiers:
    log_file = f"/home/rospi/Desktop/test_ws/src/post/post_scripts/logs/log-{identifier}.csv"
    graveyard_file = f"/home/rospi/Desktop/test_ws/graveyard/graveyard-{identifier}.csv"
    # Check if both files exist
    if os.path.exists(log_file) and os.path.exists(graveyard_file):
        df_transactions = pd.read_csv(log_file)
        df_graveyard = pd.read_csv(graveyard_file)
        # Convert nanoseconds if needed
        df_transactions['TIMESTAMP_SECONDS'] = df_transactions['TIMESTAMP'] // 1_000_000_000  # Convert to seconds
        df_graveyard['TIMESTAMP_SECONDS'] = df_graveyard['TIMESTAMP'] // 1_000_000_000  # Convert to seconds
        df_transactions['TIMESTAMP_MILLISECONDS'] = df_transactions['TIMESTAMP'] // 1_000_000  # Convert to milliseconds
        df_graveyard['TIMESTAMP_MILLISECONDS'] = df_graveyard['TIMESTAMP'] // 1_000_000  # Convert to milliseconds
        if df_transactions['TIMESTAMP'].dtype == 'int64':
            df_transactions['TIMESTAMP'] = pd.to_datetime(df_transactions['TIMESTAMP'], unit='ns')

        if df_graveyard['TIMESTAMP'].dtype == 'int64':
            df_graveyard['TIMESTAMP'] = pd.to_datetime(df_graveyard['TIMESTAMP'], unit='ns')
        df_all = pd.concat([df_transactions, df_graveyard], ignore_index=True)
        #PARCELS LOST / COMPLETED OVER TIME
        transaction_ids = set(df_transactions['MSGID'])
        graveyard_ids = set(df_graveyard['MSGID'])
        missing_ids = transaction_ids - graveyard_ids
        # Get last seen timestamp and count for each missing parcel
        df_missing = df_transactions[df_transactions['MSGID'].isin(missing_ids)]
        df_last_seen = df_missing.loc[df_missing.groupby('MSGID')['TIMESTAMP'].idxmax()][
            ['MSGID', 'TIMESTAMP', 'PINAME']
        ].reset_index(drop=True)
        df_last_seen['IN_GRAVEYARD'] = False
        # Get last seen timestamp and count for parcels in graveyard
        df_graveyard_last = df_graveyard.loc[df_graveyard.groupby('MSGID')['TIMESTAMP'].idxmax()][
            ['MSGID', 'TIMESTAMP', 'PINAME']
        ].reset_index(drop=True)
        df_graveyard_last['IN_GRAVEYARD'] = True
        # Combine results
        df_parcel_status = pd.concat([df_last_seen, df_graveyard_last], ignore_index=True)
        # Calculate lost parcels over time
        df_lost_over_time = df_parcel_status.copy()
        df_lost_over_time = df_lost_over_time.sort_values(['TIMESTAMP'])
        # Only increment counter for lost parcels (not in graveyard)
        lost_count = 0
        lost_counts = []
        completed_count = 0
        completed_counts = []
        for _, row in df_lost_over_time.iterrows():
            if not row['IN_GRAVEYARD']:
                lost_count += 1
            else:
                completed_count += 1
            lost_counts.append(lost_count)
            completed_counts.append(completed_count)
        df_lost_over_time['COMPLETED_COUNT'] = completed_counts
        df_lost_over_time['LOST_COUNT'] = lost_counts

        #TIME AT STATION
        df_tas = df_all[['MSGID', 'TIMESTAMP_MILLISECONDS', 'TIMESTAMP', 'PINAME']].copy()
        df_tas = df_tas.sort_values(['MSGID', 'TIMESTAMP_MILLISECONDS'])
        df_tas['PREV_TIMESTAMP'] = df_tas.groupby('MSGID')['TIMESTAMP_MILLISECONDS'].shift(1)
        df_tas['TIME_AT_STATION'] = df_tas['TIMESTAMP_MILLISECONDS'] - df_tas['PREV_TIMESTAMP']
        df_tas['TIME_AT_STATION'] = df_tas['TIME_AT_STATION'].fillna(0)
        #JITTER
        df_jitter = df_transactions[['MSGID','TIMESTAMP','TIMESTAMP_MILLISECONDS', 'PINAME']].copy()
        df_jitter = df_jitter.sort_values(['PINAME', 'TIMESTAMP_MILLISECONDS'])
        df_jitter['PREV_TIMESTAMP'] = df_jitter.groupby('PINAME')['TIMESTAMP_MILLISECONDS'].shift(1)
        df_jitter['JITTER'] = df_jitter['TIMESTAMP_MILLISECONDS'] - df_jitter['PREV_TIMESTAMP']
        df_jitter['JITTER'] = df_jitter['JITTER'].fillna(0)
        df_pps = df_all[['MSGID','TIMESTAMP_SECONDS', 'TIMESTAMP', 'PINAME']].copy()
        df_rate = (
            df_pps
            .groupby(['TIMESTAMP_SECONDS', 'PINAME'])
            .size()
            .reset_index(name='PARCELS_PER_SECOND')
        )
        df_pps = df_pps.merge(df_rate, on=['TIMESTAMP_SECONDS', 'PINAME'], how='left')
        df_pps = df_pps.drop_duplicates(subset=['TIMESTAMP_SECONDS', 'PINAME'])
        
        #LATENCY

        #PARCEL LIFE SPAN
        df_pls = df_all[['MSGID', 'TIMESTAMP_SECONDS', 'TIMESTAMP', 'PINAME']].copy()

        # TIME DISTRIBUTION OF LOST PARCELS(PARCELS LOST PER SECOND)

        

        # Merge all data together
        # CPU/TEMP/MEMORY/NETWORK/USAGE USAGE
        df_combined = df_all[['MSGID','TIMESTAMP_SECONDS', 'TIMESTAMP', 'PINAME','CPU_PERCENT','CPU_TEMP','RAM_PERCENT','BYTES_SENT_MB','BYTES_RECV_MB']]
        df_combined = df_combined.merge(df_lost_over_time[['MSGID','TIMESTAMP','LOST_COUNT', 'COMPLETED_COUNT']], on=['TIMESTAMP','MSGID'], how='left')
        df_combined = df_combined.merge(df_tas[['MSGID', 'TIME_AT_STATION', 'TIMESTAMP']], on=['TIMESTAMP', 'MSGID'], how='left')
        df_combined = df_combined.merge(df_jitter[['MSGID', 'JITTER', 'TIMESTAMP']], on=['TIMESTAMP', 'MSGID'], how='left')
        df_combined = df_combined.merge(df_pps[['TIMESTAMP_SECONDS', 'PINAME', 'PARCELS_PER_SECOND']], on=['TIMESTAMP_SECONDS', 'PINAME'], how='left')
        df_combined = df_combined.drop('TIMESTAMP_SECONDS', axis=1)
        df_combined.to_csv(f"/var/lib/Logsforgrafana/parcel_analysis_{identifier}.csv", index=False)
    # Create DataFrame with filenames
    all_files = [f for f in os.listdir("/var/lib/Logsforgrafana/") if f != "filenames.csv"]
    df_filenames = pd.DataFrame(all_files, columns=['filenames'])

    # Save to filenames.csv
    df_filenames.to_csv(os.path.join("/var/lib/Logsforgrafana/", "filenames.csv"), index=False)