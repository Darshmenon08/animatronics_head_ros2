#!/usr/bin/env python3
"""
Neural Network Training Script for Blendshape to Motor Mapping
Trains a model to map blendshape inputs to motor outputs.
"""

import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout
from sklearn.model_selection import train_test_split
import numpy as np
import os


def train_model(blendshapes_path: str, motors_path: str, output_path: str, epochs: int = 100):
    """
    Train a neural network to map blendshapes to motor values.
    
    Args:
        blendshapes_path: Path to blendshapes input numpy file
        motors_path: Path to motor output numpy file
        output_path: Path to save the trained model
        epochs: Number of training epochs
    """
    # Load data
    print(f"Loading data from {blendshapes_path} and {motors_path}")
    inputs = np.load(blendshapes_path)   # Shape: (num_samples, num_blendshapes)
    outputs = np.load(motors_path)        # Shape: (num_samples, num_motors)
    
    print(f"Input shape: {inputs.shape}, Output shape: {outputs.shape}")
    
    # Split the dataset
    X_train, X_test, y_train, y_test = train_test_split(
        inputs, outputs, test_size=0.2, random_state=42
    )
    
    print(f"Training samples: {len(X_train)}, Test samples: {len(X_test)}")
    
    # Build the model
    model = Sequential([
        Dense(128, activation='relu', input_dim=inputs.shape[1]),
        Dropout(0.2),
        Dense(64, activation='relu'),
        Dropout(0.2),
        Dense(32, activation='relu'),
        Dense(outputs.shape[1], activation='linear')
    ])
    
    # Compile the model
    model.compile(optimizer='adam', loss='mse', metrics=['mae'])
    
    # Print model summary
    model.summary()
    
    # Train the model
    print(f"\nTraining for {epochs} epochs...")
    history = model.fit(
        X_train, y_train,
        epochs=epochs,
        validation_split=0.2,
        batch_size=16,
        verbose=1
    )
    
    # Evaluate the model
    loss, mae = model.evaluate(X_test, y_test, verbose=0)
    print(f"\nTest Loss: {loss:.4f}, Test MAE: {mae:.4f}")
    
    # Save the model
    model.save(output_path)
    print(f"Model saved to {output_path}")
    
    return model, history


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Train blendshape to motor neural network')
    parser.add_argument('--blendshapes', type=str, required=True,
                        help='Path to blendshapes input numpy file')
    parser.add_argument('--motors', type=str, required=True,
                        help='Path to motor output numpy file')
    parser.add_argument('--output', type=str, default='blendshape_to_motor_model.h5',
                        help='Path to save the trained model')
    parser.add_argument('--epochs', type=int, default=100,
                        help='Number of training epochs')
    
    args = parser.parse_args()
    
    train_model(args.blendshapes, args.motors, args.output, args.epochs)


if __name__ == '__main__':
    main()
