"""
Scenarios adjusted for fully dynamic normalization.
Weights are relative to the maximums found in the specific session.
"""

SCENARIOS = {
    "Short Stay (10-40m)": {
        "description": "Errands, Parents - quickly find a spot near the target.",
        "params": {
            "lambda_w": 4.0,  # штраф за пешую ходьбу 8 4
            "lambda_e": 1.0,  # важность быстрого выезда 1
            "lambda_tr": 3.0,  # важность времени между парковками 3
            "lambda_turns_arr": 0.0,  # NEW
            "lambda_turns_exit": 0.0,  # NEW
            "lambda_cross": 25.0,  # NEW
            "k": 3,  # глубина цепочки парковок
            "traffic_multiplier_drive": 1.4,
            "traffic_multiplier_exit": 1.6,
        },
    },
    "Mid Stay (1-2h)": {
        "description": "Service, Tourists - find a reliable spot nearby.",
        "params": {
            "lambda_w": 5.0,
            "lambda_e": 2.0,
            "lambda_tr": 6.0,
            "lambda_turns_arr": 0.0,  # NEW 4
            "lambda_turns_exit": 0.0,  # NEW 3
            "lambda_cross": 3.0,  # NEW
            "k": 5,
            "traffic_multiplier_drive": 1.3,
            "traffic_multiplier_exit": 1.5,
        },
    },
    "Long Stay - Day (6-10h)": {
        "description": "Office, Students - walk is fine, exit must be fast for rush hour.",
        "params": {
            "lambda_w": 10.0,  # 6.0
            "lambda_e": 8.0,
            "lambda_tr": 5.0,  # 8.0
            "lambda_turns_arr": 2.0,  # NEW
            "lambda_turns_exit": 8.0,  # NEW
            "lambda_cross": 1.0,  # NEW
            "k": 7,  # 7
            "traffic_multiplier_drive": 1.65,
            "traffic_multiplier_exit": 1.8,
        },
    },
    "Long Stay - Night (8-12h)": {
        "description": "Residents - walk as little as possible.",
        "params": {
            "lambda_w": 10.0,  # 3
            "lambda_e": 2.0,  # 1
            "lambda_tr": 4.0,  # 5
            "lambda_turns_arr": 0.0,  # NEW
            "lambda_turns_exit": 3.0,  # NEW
            "lambda_cross": 7.0,  # NEW
            "k": 7,  # 7
            "traffic_multiplier_drive": 1.0,
            "traffic_multiplier_exit": 1.4,
        },
    },
    "Evening Stay (2-4h)": {
        "description": "Social, Events - exit time is critical, walk tolerated.",
        "params": {
            "lambda_w": 4.0,
            "lambda_e": 9.0,
            "lambda_tr": 7.0,
            "lambda_turns_arr": 0.0,  # NEW
            "lambda_turns_exit": 10.0,  # NEW
            "lambda_cross": 4.0,  # NEW
            "k": 4,
            "traffic_multiplier_drive": 1.0,
            "traffic_multiplier_exit": 1.8,
        },
    },
    "Taxi (5-10m)": {
        "description": "Drop-off - absolute proximity.",
        "params": {
            "lambda_w": 20.0,
            "lambda_e": 1.0,
            "lambda_tr": 10.5,
            "lambda_turns_arr": 0.0,  # NEW
            "lambda_turns_exit": 0.0,  # NEW
            "lambda_cross": 22.0,  # NEW
            "k": 2,
            "traffic_multiplier_drive": 1.4,
            "traffic_multiplier_exit": 1.4,
        },
    },
    "Delivery (1-4m)": {
        "description": "Courier - doorstep access.",
        "params": {
            "lambda_w": 50.0,
            "lambda_e": 1.00,
            "lambda_tr": 25.0,
            "lambda_turns_arr": 0.0,  # NEW
            "lambda_turns_exit": 0.0,  # NEW
            "lambda_cross": 22.0,  # NEW
            "k": 2,
            "traffic_multiplier_drive": 1.5,
            "traffic_multiplier_exit": 1.4,
        },
    },
    "Custom": {"description": "Manually tune parameters.", "params": {}},
}
