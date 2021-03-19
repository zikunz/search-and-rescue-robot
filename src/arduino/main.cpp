extern "C" {
void setup();
void loop();
}

int main() {
    setup();
    while (1) {
        loop();
    }
    return 0;
}
