// 左右のencoder値の符号が逆

#define left_outputA   32
#define left_outputB   33
#define right_outputA  34
#define right_outputB  35

// left encoder
int left_counter = 0; 
int left_counter_A;
int left_preoutput_A;  

// right encoder
int right_counter = 0; 
int right_counter_A;
int right_preoutput_A;  

void setup() { 
    pinMode (left_outputA,INPUT);
    pinMode (left_outputB,INPUT);
    pinMode (right_outputA,INPUT);
    pinMode (right_outputB,INPUT);
    
    Serial.begin (9600);
    // Reads the initial state of the left_outputA
    left_preoutput_A  = digitalRead(left_outputA);   
    right_preoutput_A = digitalRead(right_outputA);   
} 

void loop() { 
    update_left();
    update_right();
}

void update_left()
{
    left_counter_A = digitalRead(left_outputA); // Reads the "current" state of the back_left_outputA
    // If the previous and the current state of the back_left_outputA are different, that means a Pulse has occured
    if (left_counter_A != left_preoutput_A){     
        // If the back_left_outputB state is different to the back_left_outputA state, that means the encoder is rotating clockwise
        if (digitalRead(left_outputB) != left_counter_A) { 
        left_counter ++;
        } else {
        left_counter --;
        }
        Serial.print("left Position: ");
        Serial.println(left_counter);
    } 
    left_preoutput_A = left_counter_A; // Updates the previous state of the back_left_outputA with the current state
}

void update_right()
{
    right_counter_A = digitalRead(right_outputA); // Reads the "current" state of the back_left_outputA
    // If the previous and the current state of the back_left_outputA are different, that means a Pulse has occured
    if (right_counter_A != right_preoutput_A){     
        // If the back_left_outputB state is different to the back_left_outputA state, that means the encoder is rotating clockwise
        if (digitalRead(right_outputB) != right_counter_A) { 
        right_counter ++;
        } else {
        right_counter --;
        }
        Serial.print("right Position: ");
        Serial.println(right_counter);
    } 
    right_preoutput_A = right_counter_A; // Updates the previous state of the back_left_outputA with the current state
}