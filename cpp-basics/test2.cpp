#include <iostream>
#include <string>

class BankAccount {

private:
    std::string owner;
    double balance;


public:
    BankAccount(std::string owner, double initialBalance) {
        this->owner = owner;
        this->balance = initialBalance;
    }

    void deposit(double amount) {
        if (amount > 0) {
            balance += amount;
            std::cout << "Deposited: $" << amount << std::endl;
        } else {
            std::cout << "Deposit amount must be positive!" << std::endl;
        }
    }

    bool withdraw(double amount) {
        if (amount > 0 && amount <= balance) {
            balance -= amount;
            std::cout << "Withdrew: $" << amount << std::endl;
            return true;
        } else {
            std::cout << "Insufficient funds or invalid amount!" << std::endl;
            return false;
        }
    }

    double getBalance() {
        return balance;
    }

    std::string getOwner() {
        return owner;
    }
};

int main() {
    BankAccount myAccount("Kevin", 1000.0);

    std::cout << myAccount.getOwner() << "'s initial balance: $" << myAccount.getBalance() << std::endl;

    myAccount.deposit(500.0);
    std::cout << "Balance after deposit: $" << myAccount.getBalance() << std::endl;

    myAccount.withdraw(200.0);
    std::cout << "Balance after withdrawal: $" << myAccount.getBalance() << std::endl;

    myAccount.withdraw(2000.0); // Attempt to withdraw more than the balance

    return 0;
}