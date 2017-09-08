# NRF24L01-driver
射频芯片NRF24L01的基础驱动和测试程序

这份代码最早是在我读研一的时候写的，这个版本是针对51单片机的，后期的版本还开发了一份arm上的linux版本（是完全使用SPI总线的，不需要GPIO模拟SPI），最终实现arm和51单片机之间的射频通信。但是linux的版本代码已经找不到了，可惜当年没有github。只有这一份原始的单片机版本以及当时写的一篇qq日志，日志也搬家过来纪念那些年的青葱岁月啦！

```
    第一次使用无线通信的芯片，第一次接触SPI总线时序，第一次遇到芯片有问题，很多的第一次，所以最后成功了我觉得很有意义，收获了很多，有必要用寥寥数字来记录一下。
    最开始老板给的是一个NRF401的芯片，后来网上一查，这玩意早停产了，不过发现其操作和硬件接口非常简单，编程应该也非常简单，于是乎三下两下的接好电路，编好程序，上电一跑，晕了！完全没反应，一开始还怀疑自己的程序或者硬件电路哪里错了，DEBUG，查资料搞了好几天，最后确定自己没有搞错。于是乎，又把401拿掉，直接用2根导线把两块板子的串行口接好，然后奇迹出现了，如同我设想的那样运行。好吧，终于可以确定这2块不知放了多少年布满飞尘的401是坏的了。
    然后上网继续寻找替代品，淘宝上一搜人气最高的是NRF24L01，本来还想买905的，不过哥相信群众，于是买了2块NRF24L01回来。这个芯片的接口是SPI的接口，的确让我郁闷了一阵，以前从来没接触过这个玩意，而且一般的单片机哪有SPI接口，身边的一块ARM板倒是有不过只有一块啊，于是又硬着头皮去看如何用普通IO口模拟出SPI的时序来操作NRF24L01，等搞明白了芯片也寄到了，又是一阵捣鼓，接好线，相当多的线，编好程序，相对401来说很长的程序。一上电，又晕了，还是没反应。然后就是无止境的调试，修改程序，整整花了一周多时间后终于遇到了曙光啊！一个网上的高手帮我用他那边的示波器分析了一下我的程序波形，说没有问题啊，然后另一个高手告诉我应该怎样来一步一步地调试这些芯片，比如先给芯片寄存器写一个值然后马上读出来，看是否写操作正确，等等。这个经验之谈真是犹如黑夜里的一盏明灯啊，让我豁然开朗，马上照高手指点的一搞，结果出来了。2块NRF24L01的其中一块根本写不进去数据，完全是坏的，哎！人品啦！(我承认是我太傻太天真了，以为老板发货的时候都会先帮我检查一遍)马上和淘宝上的老板沟通了一下，让我把坏的寄过去他检查如果确实是他们的原因他们负责再给我寄一个过来，同时包邮费。好吧，我又等，等了一周老板来电话了通知我确实是他们的问题给我寄了一块坏的芯片，并给我重新寄来了一块好的。然后继续等啊等，又是一周，东西终于来了。这次哥颤抖着以迅雷不及掩耳盗铃儿响叮当之势再一次接好线，下好以前写好的程序，谢天谢地谢亚龙啊，这次总算测试通过，两块芯片都是好的了。
    今天自己又写了一个发送与接收单工通信的测试程序，跑了一下。恩，效果不错，看到接收端不停得把收到的数据显示在电脑上时内牛满面啊！这其中也有个小插曲，一开始接收的数组被我定义在了code段，结果发送端都能正确收到接收的应答信号，但接收端就是不能把数据写进接收数组里面，程序编译也通过。后来一想才明白CODE内的代码是下载到flash中的，是修改不了的。最后改成data修饰，终于能正常运行了。当初我曾请一个调出来的网友把他的程序借我参考下，但他说这是公司的商业机密，不能给，哎，为了以后的同仁少走弯路，我把我调好的测试程序放上来。
```
