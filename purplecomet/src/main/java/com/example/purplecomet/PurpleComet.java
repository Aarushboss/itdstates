package com.example.purplecomet;
import java.math.BigInteger;
import java.util.Scanner;
public class PurpleComet {
    public static void main(String[] args)
    {
    }
}

class problem1 {

    public static void main(String[] args)
    {
        Scanner scanner = new Scanner(System.in);


        double f;
        double c;


        System.out.print("fahrenheit:");
        f = scanner.nextDouble();

        c =(f - 32)*5/9;
        System.out.println("amount in celsius: "+c+"");

    }
}

class problem2 {

    public static void main(String[] args)
    {

        /** Find the number of digits you would write if you wrote down all of the integers from 1 through 2024:
        1,2,3,4,5,6,7,8,9,10,11,...,2022,2023,2024. **/

        int ones, tens, hundreds, thousands;
        int count = 0;

        ones = (10 - 1) * 1;
        tens = (99 - 9) * 2;
        hundreds = (999 - 99) * 3;
        thousands = (2024 - 999) * 4;

        count = ones + tens + hundreds + thousands;

        System.out.println(count);


    }
}

class problem3 {

    public static void main(String[] args)
    {

        /**One year a manufacturer announced a 10% price increase, and the cost of their product went up by $40.
         The next year the manufacturer announced a 15% price increase. Find the additional number of dollars the
         cost went up the second year.**/

        double originalcost;
        double y1cost;
        double y2cost;
        double y1increase = 40;
        double y2increase;

        y1cost = 440;
        y2cost = y1cost * 1.15;

        y2increase = y2cost - y1cost;

        System.out.println(y2increase);




    }
}

class problem4 {

    public static void main(String[] args)
    {
        /** Children numbered 1,2,3,...,400 sit around a circle in that order. Starting with child numbered 148, you
        tap the heads of children 148,139,130,..., tapping the heads of every ninth child as you walk around the
        circle. Find the number of the 100th child whose head you will tap. **/

        int sum = 148;

        for (int i = 1; i < 100; i++) {
            sum = sum - 9;
        }

        sum = sum + 800;

        System.out.println(sum);


    }
}

class problem5 {

    public static void main(String[] args)
    {

        /** Find the remainder when 2^888 + 5^888 is divided by 2024. **/

        int base1 = 2;
        int base2 = 5;
        int exponent = 888;
        int divisor = 2024;

        // Use BigInteger for large numbers
        BigInteger bigBase1 = BigInteger.valueOf(base1);
        BigInteger bigBase2 = BigInteger.valueOf(base2);
        BigInteger bigExponent = BigInteger.valueOf(exponent);
        BigInteger bigDivisor = BigInteger.valueOf(divisor);

        // Calculate the powers using BigInteger.pow()
        BigInteger bigResult1 = bigBase1.pow(exponent);
        BigInteger bigResult2 = bigBase2.pow(exponent);

        // Add the results using BigInteger.add()
        BigInteger bigDividend = bigResult1.add(bigResult2);

        // Calculate the remainder using BigInteger.mod()
        BigInteger bigRemainder = bigDividend.mod(bigDivisor);


        System.out.println("remainder:" + bigRemainder);
        System.out.println("dividend:" + bigDividend);



    }
}

class problem6 {

    public static void main(String[] args)
    {

        //1 + 2 × 3 + 4 × 5 + 6

        int sum1 = (1 + 2) * 3 + 4 * 5 + 6;
        int sum2 = (1 + 2 * 3) + 4 * 5 + 6;
        int sum3 = (1 + 2 * 3 + 4) * 5 + 6;
        int sum4 = (1 + 2 * 3 + 4 * 5) + 6;
        int sum5 = (1 + 2 * 3 + 4 * 5 + 6);
        int sum6 = 1 + (2 * 3) + 4 * 5 + 6;
        int sum7 = 1 + (2 * 3 + 4) * 5 + 6;
        int sum8 = 1 + (2 * 3 + 4 * 5) + 6;
        int sum9 = 1 + (2 * 3 + 4 * 5 + 6);
        int sum10 = 1 + 2 * (3 + 4) * 5 + 6;
        int sum11 = 1 + 2 * (3 + 4 * 5) + 6;
        int sum12 = 1 + 2 * (3 + 4 * 5 + 6);
        int sum13 = 1 + 2 * 3 + (4 * 5) + 6;
        int sum14 = 1 + 2 * 3 + (4 * 5 + 6);
        int sum15 = 1 + 2 * 3 + 4 * (5 + 6);


        System.out.println("sum 1:" + sum1);
        System.out.println("sum 2:" + sum2);
        System.out.println("sum 3:" + sum3);
        System.out.println("sum 4:" + sum4);
        System.out.println("sum 5:" + sum5);
        System.out.println("sum 6:" + sum6);
        System.out.println("sum 7:" + sum7);
        System.out.println("sum 8:" + sum8);
        System.out.println("sum 9:" + sum9);
        System.out.println("sum 10:" + sum10);
        System.out.println("sum 11:" + sum11);
        System.out.println("sum 12:" + sum12);
        System.out.println("sum 13:" + sum13);
        System.out.println("sum 14:" + sum14);
        System.out.println("sum 15:" + sum15);



    }
}

class problem7 {

    public static void main(String[] args)
    {

        //In a room there are 144 people. They are joined by n other people who are each carrying k coins. When
        //these coins are shared among all n + 144 people, each person has 2 of these coins. Find the minimum
        //possible value of 2n+ k.

        int ppl = 144;
        int n = 2;
        int k = 2;

        int totalk;

        for (int i = 0; i < 70; i++) {
            n = i;
            k = (144 - n) / 2;
            totalk = 2 * (n + ppl);
        }

//        int sum1 =
//        int sum2 =
//        int sum3 =
//        int sum4 =
//        int sum5 =
//        int sum6 =
//        int sum7 =
//        int sum8 =
//        int sum9 =
//        int sum10 =
//        int sum11 =
//        int sum12 =
//        int sum13 =
//        int sum14 =
//        int sum15 =
//
//
//        System.out.println("sum 1:" + sum1);
//        System.out.println("sum 2:" + sum2);
//        System.out.println("sum 3:" + sum3);
//        System.out.println("sum 4:" + sum4);
//        System.out.println("sum 5:" + sum5);
//        System.out.println("sum 6:" + sum6);
//        System.out.println("sum 7:" + sum7);
//        System.out.println("sum 8:" + sum8);
//        System.out.println("sum 9:" + sum9);
//        System.out.println("sum 10:" + sum10);
//        System.out.println("sum 11:" + sum11);
//        System.out.println("sum 12:" + sum12);
//        System.out.println("sum 13:" + sum13);
//        System.out.println("sum 14:" + sum14);
//        System.out.println("sum 15:" + sum15);
//







    }
}

class problem8 {

    public static void main(String[] args)
    {



    }
}

class problem9 {

    public static void main(String[] args)
    {



    }
}

class problem10 {

    public static void main(String[] args)
    {



    }
}

class problem11 {

    public static void main(String[] args)
    {



    }
}

class problem12 {

    public static void main(String[] args)
    {



    }
}

class problem13 {

    public static void main(String[] args)
    {



    }
}

class problem14 {

    public static void main(String[] args)
    {



    }
}

class problem15 {

    public static void main(String[] args)
    {



    }
}

class problem16 {

    public static void main(String[] args)
    {



    }
}

class problem17 {

    public static void main(String[] args)
    {



    }
}

class problem18 {

    public static void main(String[] args)
    {



    }
}

class problem19 {

    public static void main(String[] args)
    {



    }
}

class problem20 {

    public static void main(String[] args)
    {



    }
}

class problem21 {

    public static void main(String[] args)
    {



    }
}

class problem22 {

    public static void main(String[] args)
    {



    }
}

class problem23 {

    public static void main(String[] args)
    {



    }
}

class problem24 {

    public static void main(String[] args)
    {



    }
}

class problem25 {

    public static void main(String[] args)
    {



    }
}

class problem26 {

    public static void main(String[] args)
    {



    }
}

class problem27 {

    public static void main(String[] args)
    {



    }
}

class problem28 {

    public static void main(String[] args)
    {
        int m = 5, n = 5, p = 5;

        for (int i = 0; i < 50; i++) {

        }



    }
}

class problem29 {

    public static void main(String[] args)
    {

        double myint = 882322;

//
        for (int divisor = 1; divisor < 2000; divisor++) {
            double sum = myint/divisor;
            System.out.println(sum);
            System.out.println();
            System.out.println(divisor);
            System.out.println();


        }



    }
}

class problem30 {

    public static void main(String[] args)
    {

        int n;
        int finals;

        for (n = 1; n < 10000; n++) {
            int sum = (n*n) + 2025;
            System.out.print( Math.sqrt(sum) );
            System.out.println();
            System.out.print(n);
            System.out.println();
        }

        //  String poem = "As I was walking down the stair,\n"
        //  + " I met a man who wasn’t there.\n"
        //  + "He wasn’t there again today.\n"
        //  + " I wish, I wish he’d go away!\n";



    }
}

