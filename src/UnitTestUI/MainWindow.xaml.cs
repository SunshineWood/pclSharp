using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using UnitTest;

namespace UnitTestUI
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        private UnitTest1 test1= new UnitTest1();

        private void ButtonBase_OnClick(object sender, RoutedEventArgs e)
        {
            test1.FitCircleTest();
        }

        private void StatisticalFilter_OnClick(object sender, RoutedEventArgs e)
        {
            test1.StatisticalOutlierFilterTest();
        }

        private void SmoothFilter_OnClick(object sender, RoutedEventArgs e)
        {
            test1.SmoothFilterTest();
        }
    }
}
