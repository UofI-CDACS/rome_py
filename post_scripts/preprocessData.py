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
        
        # Fix: Handle the timedelta calculation properly
        time_diff = df_all['TIMESTAMP'] - df_all['PREV_TIMESTAMP']
        # Convert to milliseconds only where both timestamps exist
        df_all['TIME_AT_STATION'] = time_diff.dt.total_seconds() * 1000
        
        # Remove first entry for each MSGID (no previous timestamp)
        df_station_times = df_all[df_all['TIME_AT_STATION'].notna()]

        # Calculate jitter - time between transactions at the same station
        df_jitter = df_all.copy()
        df_jitter = df_jitter.sort_values(['STATION', 'TIMESTAMP'])
        df_jitter['NEXT_TIMESTAMP'] = df_jitter.groupby('STATION')['TIMESTAMP'].shift(-1)
        
        # Fix: Handle the jitter calculation properly
        jitter_diff = df_jitter['NEXT_TIMESTAMP'] - df_jitter['TIMESTAMP']
        df_jitter['JITTER'] = jitter_diff.dt.total_seconds() * 1000
        
        # Remove last entry for each station (no next timestamp)
        df_jitter = df_jitter[df_jitter['JITTER'].notna()]
        
        df_parcels_per_second = df_all.copy()
        # Calculate parcels per second for each station
        df_parcels_per_second['SECOND'] = df_parcels_per_second['TIMESTAMP'].dt.floor('S')
        parcels_per_second = df_parcels_per_second.groupby(['STATION', 'SECOND']).size().reset_index(name='PARCELS_PER_SECOND')
        
        # Get average parcels per second for each station
        avg_parcels_per_second = parcels_per_second.groupby('STATION')['PARCELS_PER_SECOND'].mean().reset_index()
        avg_parcels_per_second.rename(columns={'PARCELS_PER_SECOND': 'AVG_PARCELS_PER_SECOND'}, inplace=True)

        df_parcel_lifetime = df_all.copy()
        # Calculate lifetime of each parcel
        df_parcel_lifetime['LIFETIME'] = (df_parcel_lifetime.groupby('MSGID')['TIMESTAMP'].transform('max') - df_parcel_lifetime.groupby('MSGID')['TIMESTAMP'].transform('min')).dt.total_seconds() * 1000
        df_parcel_lifetime = df_parcel_lifetime[['MSGID', 'LIFETIME']].drop_duplicates()
        
        # Calculate parcels lost per second
        parcels_lost_per_second = df_lost_over_time.copy()
        parcels_lost_per_second['SECOND'] = parcels_lost_per_second['TIMESTAMP'].dt.floor('S')
        parcels_lost_per_second = parcels_lost_per_second.groupby(['IN_GRAVEYARD', 'SECOND']).agg({'LOST_COUNT': 'max'}).reset_index()
        parcels_lost_per_second.rename(columns={'LOST_COUNT': 'PARCELS_LOST_PER_SECOND'}, inplace=True)
        
        # Merge additional system metrics by MSGID and TIMESTAMP
        system_metrics = ['CPU_PERCENT', 'CPU_TEMP', 'RAM_PERCENT', 'BYTES_SENT_MB', 'BYTES_RECV_MB', 'PARCEL_SIZE_MB']
        df_system_metrics = df_all[['MSGID', 'TIMESTAMP'] + system_metrics].drop_duplicates()

        # Merge all data together
        df_combined = df_parcel_status.merge(df_lost_over_time[['MSGID', 'LOST_COUNT']], on='MSGID', how='left')
        df_combined = df_combined.merge(df_station_times[['MSGID', 'TIME_AT_STATION']], on='MSGID', how='left')
        df_combined = df_combined.merge(df_jitter[['STATION', 'JITTER']], on='STATION', how='left')
        df_combined = df_combined.merge(avg_parcels_per_second, on='STATION', how='left')
        df_combined = df_combined.merge(df_parcel_lifetime, on='MSGID', how='left')
        df_combined = df_combined.merge(parcels_lost_per_second, on=['IN_GRAVEYARD', 'SECOND'], how='left')
        df_combined = df_combined.merge(df_system_metrics, on=['MSGID', 'TIMESTAMP'], how='left')
        df_combined.to_csv(f"/var/lib/Logsforgrafana/combined_analysis_{identifier}.csv", index=False)

# Get all filenames in the directory and save to filenames.csv
output_dir = "/var/lib/Logsforgrafana/"
all_files = [f for f in os.listdir(output_dir) if f != "filenames.csv"]

# Create DataFrame with filenames
df_filenames = pd.DataFrame(all_files, columns=['filenames'])

# Save to filenames.csv
df_filenames.to_csv(os.path.join(output_dir, "filenames.csv"), index=False)

