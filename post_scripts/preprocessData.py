import pandas as pd
import os
import glob
import datetime as dt
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
        if df_transactions['TIMESTAMP'].dtype == 'int64':
            df_transactions['TIMESTAMP'] = pd.to_datetime(df_transactions['TIMESTAMP'], unit='ns')

        if df_graveyard['TIMESTAMP'].dtype == 'int64':
            df_graveyard['TIMESTAMP'] = pd.to_datetime(df_graveyard['TIMESTAMP'], unit='ns')
          
        # Calculate data volume per second
        df_all_messages = pd.concat([df_transactions, df_graveyard], ignore_index=True)
        df_all_messages = df_all_messages.sort_values('TIMESTAMP')
        
        # Group by second and count messages
        df_all_messages['TIMESTAMP_SECOND'] = df_all_messages['TIMESTAMP'].dt.floor('S')
        df_volume_per_second = df_all_messages.groupby('TIMESTAMP_SECOND').size().reset_index(name='MESSAGES_PER_SECOND')
        
        # Identify missing parcel IDs
        transaction_ids = set(df_transactions['MSGID'])
        graveyard_ids = set(df_graveyard['MSGID'])

        missing_ids = transaction_ids - graveyard_ids

        # Get last seen timestamp and count for each missing parcel
        df_missing = df_transactions[df_transactions['MSGID'].isin(missing_ids)]
        df_last_seen = df_missing.groupby('MSGID').agg({
            'TIMESTAMP': 'max',
            'MSGID': 'count'
        }).rename(columns={'MSGID': 'COUNT'}).reset_index()
        df_last_seen['IN_GRAVEYARD'] = False
        
        # Get last seen timestamp and count for parcels in graveyard
        df_graveyard_last = df_graveyard.groupby('MSGID').agg({
            'TIMESTAMP': 'max',
            'MSGID': 'count'
        }).rename(columns={'MSGID': 'COUNT'}).reset_index()
        df_graveyard_last['IN_GRAVEYARD'] = True

        # Combine results
        df_parcel_status = pd.concat([df_last_seen, df_graveyard_last], ignore_index=True)

        # Calculate lost parcels over time
        df_lost_over_time = df_parcel_status.copy()
        df_lost_over_time = df_lost_over_time.sort_values('TIMESTAMP')

        # Only increment counter for lost parcels (not in graveyard)
        lost_count = 0
        lost_counts = []
        for _, row in df_lost_over_time.iterrows():
            if not row['IN_GRAVEYARD']:
                lost_count += 1
            lost_counts.append(lost_count)

        df_lost_over_time['LOST_COUNT'] = lost_counts

        # Calculate station-to-station bounce times
        df_all = pd.concat([df_transactions, df_graveyard], ignore_index=True)
        df_all = df_all.sort_values(['MSGID', 'TIMESTAMP'])

        # Calculate time differences between consecutive timestamps for each MSGID
        df_all['PREV_TIMESTAMP'] = df_all.groupby('MSGID')['TIMESTAMP'].shift(1)
        df_all['TIME_AT_STATION'] = (df_all['TIMESTAMP'] - df_all['PREV_TIMESTAMP']).dt.total_seconds() * 1000

        # Remove first entry for each MSGID (no previous timestamp)
        df_station_times = df_all[df_all['TIME_AT_STATION'].notna()]

        # Merge all data together
        df_combined = df_parcel_status.merge(df_lost_over_time[['MSGID', 'LOST_COUNT']], on='MSGID', how='left')
        df_combined = df_combined.merge(df_station_times[['MSGID', 'TIME_AT_STATION']], on='MSGID', how='left')
        
        # Add timestamp second for merging with volume data
        df_combined['TIMESTAMP_SECOND'] = df_combined['TIMESTAMP'].dt.floor('S')
        
        # Merge with volume per second data
        df_combined = df_combined.merge(df_volume_per_second, on='TIMESTAMP_SECOND', how='left')
        
        df_combined.to_csv(f"/var/lib/Logsforgrafana/combined_analysis_{identifier}.csv", index=False)

    # Get all filenames in the directory and save to filenames.csv
    output_dir = "/var/lib/Logsforgrafana/"
    all_files = [f for f in os.listdir(output_dir) if f != "filenames.csv"]

    # Create DataFrame with filenames
    df_filenames = pd.DataFrame(all_files, columns=['filenames'])

    # Save to filenames.csv
    df_filenames.to_csv(os.path.join(output_dir, "filenames.csv"), index=False)

