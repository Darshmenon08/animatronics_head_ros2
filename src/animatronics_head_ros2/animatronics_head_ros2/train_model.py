#!/usr/bin/env python3
"""
Training Script for Animatronics Head
Trains a neural network to map face features to motor positions.
"""

import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import MinMaxScaler
import joblib
import os
import glob
import numpy as np

def main():
    # Define paths
    base_path = os.path.expanduser('~/animatronics_head_ros2')
    data_dir = os.path.join(base_path, 'data')
    models_dir = os.path.join(base_path, 'models')
    os.makedirs(models_dir, exist_ok=True)
    
    # Load all CSV files
    csv_files = glob.glob(os.path.join(data_dir, '*.csv'))
    if not csv_files:
        print(f"No CSV files found in {data_dir}")
        return
        
    print(f"Found {len(csv_files)} CSV files. Loading data...")
    df_list = []
    for f in csv_files:
        try:
            df_temp = pd.read_csv(f)
            df_list.append(df_temp)
        except Exception as e:
            print(f"Error reading {f}: {e}")
            
    if not df_list:
        print("No valid data loaded.")
        return
        
    df = pd.concat(df_list, ignore_index=True)
    print(f"Total samples: {len(df)}")
    
    # Define features and targets
    feature_cols = [
        'eye_l_h', 'eye_l_v', 'eye_r_h', 'eye_r_v',
        'lid_l', 'lid_r',
        'brow_l_in', 'brow_l_out', 'brow_r_in', 'brow_r_out',
        'mouth_v', 'mouth_l_h', 'mouth_r_h', 'mouth_l_v', 'mouth_r_v'
    ]
    
    target_cols = [
        "left_lid", "right_lid", "left_h", "left_v", "right_h", "right_v",
        "left_brow_1", "left_brow_2", "right_brow_1", "right_brow_2",
        "lip_up_1", "lip_up_2", "lip_up_3",
        "lip_down_1", "lip_down_2", "lip_down_3",
        "left_cheek_down", "left_cheek_up",
        "right_cheek_down", "right_cheek_up",
        "jaw"
    ]
    
    X = df[feature_cols].values
    y = df[target_cols].values
    
    # Normalize data
    print("Normalizing data...")
    scaler_X = MinMaxScaler()
    scaler_y = MinMaxScaler()
    
    X_scaled = scaler_X.fit_transform(X)
    y_scaled = scaler_y.fit_transform(y)
    
    # Save scalers for inference
    joblib.dump(scaler_X, os.path.join(models_dir, 'scaler_X.pkl'))
    joblib.dump(scaler_y, os.path.join(models_dir, 'scaler_y.pkl'))
    
    # Split data
    X_train, X_test, y_train, y_test = train_test_split(X_scaled, y_scaled, test_size=0.2, random_state=42)
    
    # Build model
    print("Building model...")
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(64, activation='relu', input_shape=(X_train.shape[1],)),
        tf.keras.layers.Dense(64, activation='relu'),
        tf.keras.layers.Dense(y_train.shape[1])
    ])
    
    model.compile(optimizer='adam', loss='mean_squared_error')
    
    # Train model
    print("Training model...")
    history = model.fit(
        X_train, y_train,
        epochs=100,
        batch_size=32,
        validation_data=(X_test, y_test),
        verbose=1
    )
    
    # Save model
    model_path = os.path.join(models_dir, 'face_model.h5')
    model.save(model_path)
    print(f"Model saved to {model_path}")
    
    # Evaluate
    loss = model.evaluate(X_test, y_test)
    print(f"Test Loss: {loss}")

if __name__ == '__main__':
    main()
