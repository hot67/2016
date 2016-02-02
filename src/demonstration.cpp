#include <iostream>
#include <string>
using namespace std;

class Foods {
public:
	bool iseaten;
	string food;
	Foods(string typeoffood) {
		iseaten = false;
		food = typeoffood;
	}
	string GetFood();
	void PrintFood();
	bool IsEaten();
	void Eat();
};

string Foods::GetFood() {
	return food;
}

void Foods::PrintFood() {
	cout << food;
}

bool Foods::IsEaten() {
	return iseaten;
}

void Foods::Eat() {
	iseaten = true;
}

int main() {
	Foods orange("orange");
	Foods* AnotherOrange;

	AnotherOrange = &orange;
	AnotherOrange->PrintFood();

	orange.Eat(); //Eat the orange
	bool stuff = AnotherOrange->iseaten(); //Did someone eat the orange?

	Foods apple ("apple");
	Foods* AnotherApple;
/*
	AnotherApple = &apple;
	AnotherApple->PrintFood();
*/
	apple.Eat();
	cout << AnotherApple->iseaten();


}
