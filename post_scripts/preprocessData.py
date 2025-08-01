import pandas as pd
import os
import glob
import datetime as dt
import gc

# Get all log files and their identifiers
log_files = glob.glob("/home/rospi/Desktop/test_ws/src/post/post_scripts/logs/log-*.csv")
identifiers = [os.path.basename(f).replace("log-", "").replace(".csv", "") for f in log_files]

print(f"Found {len(identifiers)} log files to process")

# Process each identifier
for i, identifier in enumerate(identifiers):
    print(f"Processing {i+1}/{len(identifiers)}: {identifier}")
    
    log_file = f"/home/rospi/Desktop/test_ws/src/post/post_scripts/logs/log-{identifier}.csv"
    graveyard_file = f"/home/rospi/Desktop/test_ws/graveyard/graveyard-{identifier}.csv"
    
    # Check if both files exist
    if os.path.exists(log_file) and os.path.exists(graveyard_file):
        try:
            # Read CSV files with header row
            df_transactions = pd.read_csv(log_file)
            df_graveyard = pd.read_csv(graveyard_file)
            
            # Skip empty files
            if df_transactions.empty or df_graveyard.empty:
                print(f"Skipping {identifier} - empty files")
                continue
            
            # Convert TIMESTAMP to datetime - handle both string and numeric formats
            def convert_timestamp(df):
                if df['TIMESTAMP'].dtype == 'object':  # String format
                    df['TIMESTAMP'] = pd.to_numeric(df['TIMESTAMP'], errors='coerce')
                    df['TIMESTAMP'] = pd.to_datetime(df['TIMESTAMP'], unit='ns')
                elif df['TIMESTAMP'].dtype in ['int64', 'float64']:  # Numeric format
                    df['TIMESTAMP'] = pd.to_datetime(df['TIMESTAMP'], unit='ns')
                return df
            
            df_transactions = convert_timestamp(df_transactions)
            df_graveyard = convert_timestamp(df_graveyard)
            
            # Remove rows with invalid timestamps
            df_transactions = df_transactions.dropna(subset=['TIMESTAMP'])
            df_graveyard = df_graveyard.dropna(subset=['TIMESTAMP'])
            
            if df_transactions.empty or df_graveyard.empty:
                print(f"Skipping {identifier} - no valid timestamps")
                continue
            
            print(f"Processing {len(df_transactions)} transactions and {len(df_graveyard)} graveyard entries")
              
            # Identify missing parcel IDs
            transaction_ids = set(df_transactions['MSGID'])
            graveyard_ids = set(df_graveyard['MSGID'])
            missing_ids = transaction_ids - graveyard_ids
            
            # Create simplified combined analysis
            df_combined_simple = pd.DataFrame()
            
            # Basic statistics
            total_transactions = len(df_transactions)
            total_graveyard = len(df_graveyard)
            missing_count = len(missing_ids)
            
            # Time range
            start_time = min(df_transactions['TIMESTAMP'].min(), df_graveyard['TIMESTAMP'].min())
            end_time = max(df_transactions['TIMESTAMP'].max(), df_graveyard['TIMESTAMP'].max())
            duration_seconds = (end_time - start_time).total_seconds()
            
            # Calculate basic metrics only
            df_all = pd.concat([df_transactions, df_graveyard], ignore_index=True)
            df_all = df_all.sort_values(['MSGID', 'TIMESTAMP'])
            
            # Calculate simple bounce times for a sample of messages (to avoid memory issues)
            sample_size = min(1000, len(df_all))
            df_sample = df_all.head(sample_size).copy()
            df_sample['PREV_TIMESTAMP'] = df_sample.groupby('MSGID')['TIMESTAMP'].shift(1)
            
            # Calculate time differences
            mask = df_sample['PREV_TIMESTAMP'].notna()
            df_sample.loc[mask, 'TIME_AT_STATION'] = (
                df_sample.loc[mask, 'TIMESTAMP'] - df_sample.loc[mask, 'PREV_TIMESTAMP']
            ).dt.total_seconds() * 1000
            
            avg_bounce_time = df_sample['TIME_AT_STATION'].mean() if not df_sample['TIME_AT_STATION'].isna().all() else 0
            
            # Create summary report
            summary = {
                'identifier': identifier,
                'total_transactions': total_transactions,
                'total_graveyard': total_graveyard,
                'missing_parcels': missing_count,
                'loss_rate': missing_count / total_transactions if total_transactions > 0 else 0,
                'start_time': start_time,
                'end_time': end_time,
                'duration_seconds': duration_seconds,
                'avg_bounce_time_ms': avg_bounce_time,
                'throughput_per_second': total_transactions / duration_seconds if duration_seconds > 0 else 0
            }
            
            # Save summary instead of full analysis
            summary_df = pd.DataFrame([summary])
            summary_df.to_csv(f"/var/lib/Logsforgrafana/summary_{identifier}.csv", index=False)
            
            print(f"Completed {identifier}: {total_transactions} transactions, {missing_count} missing ({missing_count/total_transactions*100:.1f}% loss)")
            
            # Force garbage collection
            del df_transactions, df_graveyard, df_all, df_sample
            gc.collect()
            
        except Exception as e:
            print(f"Error processing {identifier}: {e}")
            continue
    else:
        print(f"Missing files for {identifier}")

print("Creating filenames list...")

# Get all filenames in the directory and save to filenames.csv
output_dir = "/var/lib/Logsforgrafana/"
try:
    all_files = [f for f in os.listdir(output_dir) if f != "filenames.csv"]
    
    # Create DataFrame with filenames
    df_filenames = pd.DataFrame(all_files, columns=['filenames'])
    
    # Save to filenames.csv
    df_filenames.to_csv(os.path.join(output_dir, "filenames.csv"), index=False)
    print(f"Created filenames.csv with {len(all_files)} files")
except Exception as e:
    print(f"Error creating filenames.csv: {e}")

print("Processing complete!")

