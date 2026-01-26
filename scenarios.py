"""
Scenarios adjusted for fully dynamic normalization.
Weights are relative to the maximums found in the specific session.
"""

SCENARIOS = {
    "Short Stay (10-40m)": {
        "description": "Errands, Parents - quickly find a spot near the target.",
        "params": {
            "lambda_w": 5.0,   
            "lambda_e": 0.5,   
            "lambda_tr": 3.6,  
            "k": 2,            
            "traffic_multiplier": 1.4
        }
    },
    "Mid Stay (1-2h)": {
        "description": "Service, Tourists - find a reliable spot nearby.",
        "params": {
            "lambda_w": 3.0,   
            "lambda_e": 1.2,   
            "lambda_tr": 2.1,  
            "k": 3,            
            "traffic_multiplier": 1.3
        }
    },
    "Long Stay - Day (6-10h)": {
        "description": "Office, Students - walk is fine, exit must be fast for rush hour.",
        "params": {
            "lambda_w": 30.0,   
            "lambda_e": 45.0,   
            "lambda_tr": 45.0,  
            "k": 4,            
            "traffic_multiplier": 1.7
        }
    },
    "Long Stay - Night (8-12h)": {
        "description": "Residents - walk as little as possible.",
        "params": {
            "lambda_w": 6.0,   
            "lambda_e": 0.5,   
            "lambda_tr": 5.0,  
            "k": 4,            
            "traffic_multiplier": 1.0
        }
    },
    "Evening Stay (2-4h)": {
        "description": "Social, Events - exit time is critical, walk tolerated.",
        "params": {
            "lambda_w": 1.2,   
            "lambda_e": 16.0,   
            "lambda_tr": 12.1,  
            "k": 4,            
            "traffic_multiplier": 1.0
        }
    },
    "Taxi (5-10m)": {
        "description": "Drop-off - absolute proximity.",
        "params": {
            "lambda_w": 30.0,  
            "lambda_e": 0.1,   
            "lambda_tr": 15.5,  
            "k": 1,            
            "traffic_multiplier": 1.4
        }
    },
    "Delivery (1-4m)": {
        "description": "Courier - doorstep access.",
        "params": {
            "lambda_w": 50.0,  
            "lambda_e": 0.05,  
            "lambda_tr": 25.3,  
            "k": 1,            
            "traffic_multiplier": 1.5
        }
    },
    "Custom": {
        "description": "Manually tune parameters.",
        "params": {}
    }
}